// Gaussian scene integration reuses the user's existing LCC2 renderer.
#include <GL/glew.h>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <Ogre.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include "gsplat_rviz_plugin/splat_cloud.hpp"
#include "gsplat_rviz_plugin/sorters/cpu_sorter.hpp"

namespace gazebo
{
class K2GaussianVisual : public VisualPlugin
{
  rendering::VisualPtr visual;
  event::ConnectionPtr connection;
  std::unique_ptr<gsplat_rviz_plugin::SplatCloud> cloud;
  Ogre::SceneNode *node = nullptr;
  std::filesystem::path root;
  nlohmann::json config;
  bool started = false, failed = false, rendered = false;
  uint64_t count = 0;
  std::string role;

  void Status(const std::string &state)
  {
    nlohmann::json report = {
      {"state", state}, {"role", role}, {"loaded_gaussians", count},
      {"draw_calls", cloud ? cloud->getDrawCalls() : 0},
      {"renderer", "original full-resolution EWA Gaussian splatting"},
      {"mesh_appearance_proxy", false}, {"extra_downsampling", false}};
    std::ofstream(root / ("render_status_" + role + ".json")) << report.dump(2);
  }

  void Initialize()
  {
    config = nlohmann::json::parse(std::ifstream(root / "scene_config.json"));
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
      throw std::runtime_error("No current Gazebo OpenGL context");
    while (glGetError() != GL_NO_ERROR) {}
    const std::string group = "rviz_rendering";
    auto &groups = Ogre::ResourceGroupManager::getSingleton();
    if (!groups.resourceGroupExists(group)) groups.createResourceGroup(group);
    groups.addResourceLocation((root / "shaders").string(), "FileSystem", group);
    auto &programs = Ogre::HighLevelGpuProgramManager::getSingleton();
    auto vp = programs.createProgram("k2/gaussian_vertex", group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
    vp->setSourceFile("splat.vert");
    vp->load();
    auto fp = programs.createProgram("k2/gaussian_fragment", group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
    fp->setSourceFile("splat.frag");
    fp->load();
    if (vp->hasCompileError() || fp->hasCompileError())
      throw std::runtime_error("Original Gaussian shaders failed to compile");
    auto params = vp->getDefaultParameters();
    params->setNamedAutoConstant("view_matrix", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
    params->setNamedAutoConstant("projectionMatrix", Ogre::GpuProgramParameters::ACT_PROJECTION_MATRIX);
    params->setNamedAutoConstant("viewport_size", Ogre::GpuProgramParameters::ACT_VIEWPORT_SIZE);
    fp->getDefaultParameters()->setNamedConstant("u_alpha_threshold", 1.0f / 255.0f);
    auto material = Ogre::MaterialManager::getSingleton().create("gsplat_rviz_plugin/GaussianSplat", group);
    material->setReceiveShadows(false);
    auto pass = material->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);
    pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(true);
    pass->setVertexProgram(vp->getName());
    pass->setFragmentProgram(fp->getName());
    material->load();
    if (!material->getNumSupportedTechniques())
      throw std::runtime_error("Original Gaussian material is unsupported");

    std::ifstream input(root / "gaussians.bin", std::ios::binary);
    char magic[8];
    input.read(magic, 8);
    input.read(reinterpret_cast<char *>(&count), 8);
    if (!input || std::memcmp(magic, "LCCGS001", 8) ||
        count != config.at("render_gaussians").get<uint64_t>())
      throw std::runtime_error("Original Gaussian cache is invalid");
    std::vector<gsplat_rviz_plugin::SplatGPU> splats(count);
    float data[13];
    for (uint64_t i = 0; i < count; ++i)
    {
      input.read(reinterpret_cast<char *>(data), sizeof(data));
      if (!input) throw std::runtime_error("Incomplete Gaussian cache");
      auto &splat = splats[i];
      std::copy(data, data + 3, splat.center);
      splat.alpha = data[12];
      std::copy(data + 3, data + 6, splat.covA);
      std::copy(data + 6, data + 9, splat.covB);
      for (int k = 0; k < 3; ++k)
        splat.sh[0][k] = (data[9 + k] - .5f) / .28209479177387814f;
    }
    node = visual->GetSceneNode()->getCreator()->getRootSceneNode()->createChildSceneNode();
    const auto pose = visual->WorldPose();
    node->setPosition(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    node->setOrientation(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
    cloud = std::make_unique<gsplat_rviz_plugin::SplatCloud>(node);
    cloud->setQueryFlags(0);
    cloud->setCastShadows(false);
    cloud->setSorter(std::make_unique<gsplat_rviz_plugin::CpuSorter>());
    cloud->setAlphaThreshold(1.0f / 255.0f);
    cloud->setSplats(std::move(splats), 0);
    if (auto camera = visual->GetScene()->GetUserCamera(0))
      camera->SetClipDist(.05f, 5000.f);
    Status("loaded");
    gzmsg << "K2_FULL_GAUSSIANS_LOADED role=" << role << " count=" << count << std::endl;
  }

  void Update()
  {
    if (failed) return;
    try
    {
      if (!started) { started = true; Initialize(); }
      if (!rendered && cloud && cloud->getDrawCalls() >= 10)
      {
        rendered = true;
        Status("rendered");
        gzmsg << "K2_FULL_GAUSSIANS_RENDERED role=" << role << std::endl;
      }
    }
    catch (const std::exception &error)
    {
      failed = true;
      gzerr << "K2_GAUSSIANS_ERROR role=" << role << " " << error.what() << std::endl;
      Status("error");
    }
  }

public:
  ~K2GaussianVisual() override { connection.reset(); cloud.reset(); }
  void Load(rendering::VisualPtr value, sdf::ElementPtr sdf) override
  {
    visual = value;
    root = sdf->Get<std::string>("directory");
    role = visual->GetScene()->IsServer() ? "server" : "client";
    gzmsg << "K2_GAUSSIAN_PLUGIN_LOADED role=" << role << std::endl;
    connection = event::Events::ConnectPreRender(std::bind(&K2GaussianVisual::Update, this));
  }
};
GZ_REGISTER_VISUAL_PLUGIN(K2GaussianVisual)
}  // namespace gazebo
