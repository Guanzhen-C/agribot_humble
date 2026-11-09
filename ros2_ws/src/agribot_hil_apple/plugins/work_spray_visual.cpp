#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <Ogre.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreParticleAffector.h>
#include <OgreParticleSystem.h>
#include <OgreParticleEmitter.h>
#include <OgreParticle.h>
#include <nlohmann/json.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include "spray_occlusion.hpp"

namespace agribot_hil
{
class WorkSprayVisual : public gazebo::VisualPlugin
{
public:
  ~WorkSprayVisual() override
  {
    update_.reset();
    subscription_.reset();
    depth_subscription_.reset();
    if (particles_ && visual_) {
      auto manager = visual_->GetScene()->OgreSceneManager();
      mist_node_->detachObject(particles_);
      manager->destroyParticleSystem(particles_);
      manager->destroySceneNode(mist_node_);
    }
    if (!material_.isNull() && Ogre::MaterialManager::getSingletonPtr()) {
      Ogre::MaterialManager::getSingleton().remove(material_->getName());
    }
    if (!texture_.isNull() && Ogre::TextureManager::getSingletonPtr()) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
    }
  }

  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    visual_ = visual;
    obstacle_clipping_ = sdf->Get<bool>("obstacle_clipping", false).first;
    auto manager = visual_->GetScene()->OgreSceneManager();
    const double total_rate = sdf->Get<double>("total_emission_rate", 840.).first;
    const double lifetime_min = sdf->Get<double>("lifetime_min", .65).first;
    const double lifetime_max = sdf->Get<double>("lifetime_max", .95).first;
    const double velocity_min = sdf->Get<double>("velocity_min", 1.1).first;
    const double velocity_max = sdf->Get<double>("velocity_max", 1.6).first;
    const double growth_rate = sdf->Get<double>("growth_rate", .06).first;
    const auto colour = sdf->Get<ignition::math::Color>("particle_colour",
      ignition::math::Color(.84, .94, 1., .28)).first;
    CreateMistMaterial();
    // Longer plumes need enough live particles to avoid clipping their tails.
    const auto quota = static_cast<size_t>(std::max(1800., std::ceil(total_rate*lifetime_max*1.2)));
    particles_ = manager->createParticleSystem(visual_->Name() + "_spray", quota);
    // The invisible anchor sphere has a tiny geometry scale. Its pose is useful,
    // but that scale must not collapse metre-valued nozzle positions to its origin.
    mist_node_ = visual_->GetSceneNode()->createChildSceneNode();
    mist_node_->setInheritScale(false);
    mist_node_->setScale(Ogre::Vector3::UNIT_SCALE);
    mist_node_->attachObject(particles_);
    particles_->setMaterialName(material_->getName());
    particles_->setDefaultDimensions(.020, .026);
    particles_->setSortingEnabled(true);
    particles_->setCastShadows(false);
    particles_->setKeepParticlesInLocalSpace(false);
    auto fade = particles_->addAffector("ColourFader");
    fade->setParameter("alpha", "-0.30");
    auto grow = particles_->addAffector("Scaler");
    grow->setParameter("rate", std::to_string(growth_rate));
    auto gravity = particles_->addAffector("LinearForce");
    gravity->setParameter("force_vector", "0 0 -0.25");
    gravity->setParameter("force_application", "add");
    using Vector = ignition::math::Vector3d;
    std::vector<std::pair<Vector, Vector>> nozzles;
    if (sdf->HasElement("nozzle")) {
      for (auto nozzle = sdf->GetElement("nozzle"); nozzle;
        nozzle = nozzle->GetNextElement("nozzle"))
      {
        nozzles.emplace_back(nozzle->Get<Vector>("position"), nozzle->Get<Vector>("direction"));
      }
    } else {
      // Retain the two-outlet commissioning-pad configuration.
      for (int side : {-1, 1}) {
        const std::string suffix = side < 0 ? "a" : "b";
        nozzles.emplace_back(sdf->Get<Vector>("nozzle_" + suffix, {0, side * .68, .35}).first,
          sdf->Get<Vector>("direction_" + suffix, {0, static_cast<double>(side), -.18}).first);
      }
    }
    emission_per_nozzle_ = total_rate / nozzles.size();
    const double cone_angle = sdf->Get<double>("cone_angle", .22).first;
    for (const auto & [position, direction] : nozzles) {
      auto emitter = particles_->addEmitter("Point");
      emitter->setPosition(Ogre::Vector3(position.X(), position.Y(), position.Z()));
      emitter->setDirection(Ogre::Vector3(direction.X(), direction.Y(), direction.Z()));
      emitter->setAngle(Ogre::Radian(cone_angle));
      emitter->setTimeToLive(lifetime_min, lifetime_max);
      emitter->setParticleVelocity(velocity_min, velocity_max);
      emitter->setColour(Ogre::ColourValue(colour.R(), colour.G(), colour.B(), colour.A()));
      emitter->setEmissionRate(0);
    }
    gzmsg << "Water mist visual: outlets=" << nozzles.size()
          << " speed=" << velocity_min << ".." << velocity_max
          << " lifetime=" << lifetime_min << ".." << lifetime_max
          << " cone=" << cone_angle << " rate=" << total_rate << " quota=" << quota
          << " colour=" << colour << "\n";
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(visual_->GetScene()->Name());
    subscription_ = transport_->Subscribe("~/hil/work/spray", &WorkSprayVisual::OnSpray, this);
    if (obstacle_clipping_) {
      depth_subscription_ = transport_->Subscribe("~/hil/work/spray_depth", &WorkSprayVisual::OnDepth, this);
    }
    update_ = gazebo::event::Events::ConnectPreRender([this]() {
      const auto now = std::chrono::steady_clock::now().time_since_epoch();
      const double seconds = std::chrono::duration<double>(now).count();
      std::vector<SprayDepthGrid> depth;
      if (obstacle_clipping_) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (seconds-depth_received_ < .5) {depth = depth_;}
      }
      const bool clipping_ready = !obstacle_clipping_ || depth.size() == particles_->getNumEmitters();
      const double rate = seconds-received_.load() < .5 && clipping_ready ? rate_.load() : 0.;
      if (obstacle_clipping_) {
        if (clipping_ready) {ClipParticles(depth);}
        else {particles_->clear(); particle_owners_.clear();}
      }
      if (rate > .05 && !logged_origins_) {
        gzmsg << "Water mist anchor scale=" << visual_->GetSceneNode()->_getDerivedScale()
              << " independent frame scale=" << mist_node_->_getDerivedScale() << "\n";
        for (unsigned i = 0; i < particles_->getNumEmitters(); ++i) {
          const auto world = mist_node_->_getFullTransform() * particles_->getEmitter(i)->getPosition();
          gzmsg << "Water mist outlet " << i << " world=" << world << "\n";
        }
        logged_origins_ = true;
      }
      for (unsigned i = 0; i < particles_->getNumEmitters(); ++i) {
        particles_->getEmitter(i)->setEmissionRate(rate * emission_per_nozzle_);
      }
      if (obstacle_clipping_ && seconds-last_clip_log_ > 5.) {
        gzmsg << "Spray particle clipping: culled=" << culled_ << " active="
              << particles_->getNumParticles() << " depth_fresh=" << clipping_ready << "\n";
        last_clip_log_ = seconds;
      }
    });
  }

private:
  void OnDepth(ConstGzStringPtr & message)
  {
    try {
      const auto packet = nlohmann::json::parse(message->data());
      if (packet.at("version") != 1) {return;}
      std::vector<SprayDepthGrid> depth;
      const auto vector = [](const nlohmann::json & values) {
        if (values.size() != 3) {throw std::runtime_error("Invalid spray depth vector");}
        return ignition::math::Vector3d(values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>());
      };
      for (const auto & value : packet.at("nozzles")) {
        SprayDepthGrid grid;
        grid.origin = vector(value.at("origin")); grid.direction = vector(value.at("direction"));
        grid.u = vector(value.at("u")); grid.v = vector(value.at("v"));
        grid.size = value.at("size"); grid.tangent = value.at("tangent");
        grid.ranges = value.at("ranges").get<std::vector<double>>();
        if (grid.size != 9 || grid.ranges.size() != 81 || !std::isfinite(grid.tangent) || grid.tangent <= 0. ||
          !std::all_of(grid.ranges.begin(), grid.ranges.end(), [](double x) {return std::isfinite(x) && x >= 0. && x <= 4.5;})) {return;}
        depth.push_back(std::move(grid));
      }
      std::lock_guard<std::mutex> lock(depth_mutex_);
      depth_ = std::move(depth);
      depth_received_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    } catch (const std::exception &) {
      // Invalid/stale geometry never permits an unblocked fallback plume.
    }
  }

  void ClipParticles(const std::vector<SprayDepthGrid> & depth)
  {
    std::unordered_set<Ogre::Particle *> alive;
    auto iterator = particles_->_getIterator();
    while (!iterator.end()) {
      auto particle = iterator.getNext();
      alive.insert(particle);
      const double age = particle->totalTimeToLive-particle->timeToLive;
      auto found = particle_owners_.find(particle);
      if (found == particle_owners_.end() || age+.001 < found->second.age) {
        const auto birth = particle->position-particle->direction*age;
        const ignition::math::Vector3d origin(birth.x, birth.y, birth.z);
        size_t owner = 0;
        for (size_t i = 1; i < depth.size(); ++i) {
          if ((origin-depth[i].origin).SquaredLength() < (origin-depth[owner].origin).SquaredLength()) {owner = i;}
        }
        found = particle_owners_.insert_or_assign(particle, ParticleOwner{owner, age}).first;
      }
      found->second.age = age;
      const auto & p = particle->position;
      // Include billboard growth and one depth-frame movement margin, so even
      // the visible sprite edge stops before a surface, not beyond its centre.
      if (depth[found->second.nozzle].Occluded({p.x, p.y, p.z}, .065+.05*age)) {
        particle->timeToLive = 0.; particle->colour.a = 0.; ++culled_;
      }
    }
    for (auto it = particle_owners_.begin(); it != particle_owners_.end();) {
      if (!alive.count(it->first)) {it = particle_owners_.erase(it);} else {++it;}
    }
  }

  void CreateMistMaterial()
  {
    const auto group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    const std::string name = visual_->Name() + "_water_mist";
    constexpr unsigned size = 64;
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      name + "_texture", group, Ogre::TEX_TYPE_2D, size, size, 0,
      Ogre::PF_BYTE_RGBA, Ogre::TU_STATIC_WRITE_ONLY);
    auto buffer = texture_->getBuffer();
    buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const auto & pixels = buffer->getCurrentLock();
    auto bytes = static_cast<unsigned char *>(pixels.data);
    // Soft radial alpha avoids the rectangular edges of untextured billboards.
    for (unsigned y = 0; y < size; ++y) {
      for (unsigned x = 0; x < size; ++x) {
        const double dx = (x + .5) * 2. / size - 1.;
        const double dy = (y + .5) * 2. / size - 1.;
        const double radius2 = dx * dx + dy * dy;
        const double alpha = radius2 < 1. ? std::exp(-3.5 * radius2) * (1. - radius2) : 0.;
        Ogre::PixelUtil::packColour(Ogre::ColourValue(1., 1., 1., alpha), pixels.format,
          bytes + (y * pixels.rowPitch + x) * 4);
      }
    }
    buffer->unlock();
    material_ = Ogre::MaterialManager::getSingleton().create(name, group);
    material_->setReceiveShadows(false);
    auto pass = material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);
    auto unit = pass->createTextureUnitState(texture_->getName());
    unit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    unit->setTextureFiltering(Ogre::TFO_BILINEAR);
    unit->setColourOperationEx(Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_DIFFUSE);
    unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_DIFFUSE);
    material_->load();
  }

  void OnSpray(ConstVector3dPtr & msg)
  {
    rate_.store(std::isfinite(msg->x()) ? std::clamp(msg->x(), 0., 1.) : 0.);
    received_.store(std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }
  gazebo::rendering::VisualPtr visual_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::SubscriberPtr subscription_;
  gazebo::transport::SubscriberPtr depth_subscription_;
  gazebo::event::ConnectionPtr update_;
  Ogre::ParticleSystem * particles_{nullptr};
  Ogre::SceneNode * mist_node_{nullptr};
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  double emission_per_nozzle_{0.};
  bool logged_origins_{false};
  bool obstacle_clipping_{false};
  struct ParticleOwner {size_t nozzle; double age;};
  std::unordered_map<Ogre::Particle *, ParticleOwner> particle_owners_;
  std::mutex depth_mutex_;
  std::vector<SprayDepthGrid> depth_;
  double depth_received_{-1e9}, last_clip_log_{0.};
  size_t culled_{0};
  std::atomic<double> rate_{0.}, received_{0.};
};
GZ_REGISTER_VISUAL_PLUGIN(WorkSprayVisual)
}  // namespace agribot_hil
