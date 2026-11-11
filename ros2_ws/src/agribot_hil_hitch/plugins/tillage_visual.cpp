#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <mutex>
#include <vector>
#include <Ogre.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreParticleEmitter.h>
#include <OgreParticleSystem.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include "harrow_surface.hpp"

namespace agribot_hitch {
class TillageVisual : public gazebo::VisualPlugin {
public:
  ~TillageVisual() override
  {
    update_.reset(); state_.reset(); swaths_.reset();
    if (visual_) {
      auto manager = visual_->GetScene()->OgreSceneManager();
      if (dust_) {dust_node_->detachObject(dust_); manager->destroyParticleSystem(dust_);}
      if (trail_) {trail_node_->detachObject(trail_); manager->destroyManualObject(trail_);}
      if (clods_) {trail_node_->detachObject(clods_); manager->destroyManualObject(clods_);}
      if (dust_node_) {manager->destroySceneNode(dust_node_);}
      if (trail_node_) {manager->destroySceneNode(trail_node_);}
    }
    if (Ogre::MaterialManager::getSingletonPtr()) {
      if (!dust_material_.isNull()) {Ogre::MaterialManager::getSingleton().remove(dust_material_->getName());}
      if (!soil_material_.isNull()) {Ogre::MaterialManager::getSingleton().remove(soil_material_->getName());}
    }
    if (!texture_.isNull() && Ogre::TextureManager::getSingletonPtr()) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
    }
  }
  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    visual_ = visual;
    ground_z_ = sdf->Get<double>("ground_z", -.002).first;
    roller_origin_ = sdf->Get<ignition::math::Vector3d>("roller_origin");
    width_ = sdf->Get<double>("working_width");
    bed_offset_ = sdf->Get<double>("settled_bed_offset");
    for (auto item = sdf->GetElement("disc_origin"); item; item = item->GetNextElement("disc_origin")) {
      disc_origins_.push_back(item->Get<ignition::math::Vector3d>());
    }
    if (disc_origins_.size() != 16 || width_ <= 0.) {throw std::runtime_error("Missing two-stage harrow geometry");}
    for (const auto &origin : disc_origins_) {disc_mean_x_ += origin.X()/disc_origins_.size();}
    const auto group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    std::string id = visual_->Name()+"_hitch_soil";
    texture_ = Ogre::TextureManager::getSingleton().createManual(id+"_texture", group,
      Ogre::TEX_TYPE_2D, 32, 32, 0, Ogre::PF_BYTE_RGBA, Ogre::TU_STATIC_WRITE_ONLY);
    auto buffer = texture_->getBuffer(); buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const auto &pixels = buffer->getCurrentLock(); auto bytes = static_cast<unsigned char *>(pixels.data);
    for (unsigned y = 0; y < 32; ++y) {
      for (unsigned x = 0; x < 32; ++x) {
        double radius = std::hypot((x-15.5)/15.5, (y-15.5)/15.5);
        auto index = (y*pixels.rowPitch+x)*4;
        bytes[index] = bytes[index+1] = bytes[index+2] = 255;
        bytes[index+3] = static_cast<unsigned char>(180.*std::pow(std::max(0., 1.-radius), 2));
      }
    }
    buffer->unlock();
    dust_material_ = Ogre::MaterialManager::getSingleton().create(id+"_dust", group);
    auto pass = dust_material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false); pass->setDepthWriteEnabled(false);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->createTextureUnitState(texture_->getName());
    soil_material_ = Ogre::MaterialManager::getSingleton().create(id+"_soil", group);
    pass = soil_material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(true); pass->setCullingMode(Ogre::CULL_NONE);
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE | Ogre::TVC_AMBIENT);
    pass->setSpecular(0., 0., 0., 1.);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    const std::filesystem::path soil(sdf->Get<std::string>("soil_texture"));
    if (!std::filesystem::is_regular_file(soil)) {throw std::runtime_error("Missing original scene soil texture");}
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(soil.parent_path().string(), "FileSystem", group);
    pass->createTextureUnitState(soil.filename().string())->setTextureAddressingMode(Ogre::TextureUnitState::TAM_WRAP);
    auto manager = visual_->GetScene()->OgreSceneManager();
    dust_node_ = visual_->GetSceneNode()->createChildSceneNode();
    dust_node_->setInheritScale(false);
    dust_ = manager->createParticleSystem(id+"_particles", 1000);
    dust_node_->attachObject(dust_); dust_->setMaterialName(dust_material_->getName());
    dust_->setDefaultDimensions(.055, .075); dust_->setKeepParticlesInLocalSpace(false);
    dust_->setCastShadows(false); dust_->setSortingEnabled(true);
    auto fade = dust_->addAffector("ColourFader"); fade->setParameter("alpha", "-0.45");
    auto growth = dust_->addAffector("Scaler"); growth->setParameter("rate", ".10");
    auto gravity = dust_->addAffector("LinearForce"); gravity->setParameter("force_vector", "0 0 -0.8");
    for (const auto &origin : disc_origins_) {
      auto emitter = dust_->addEmitter("Point");
      emitter->setPosition({static_cast<float>(origin.X()), static_cast<float>(origin.Y()),
                           static_cast<float>(origin.Z()-.20)});
      emitter->setDirection({-.3f, 0.f, 1.f}); emitter->setAngle(Ogre::Radian(.5));
      emitter->setParticleVelocity(.20, .45); emitter->setTimeToLive(.35, .65);
      emitter->setColour(Ogre::ColourValue(.45f, .36f, .25f, .45f)); emitter->setEmissionRate(0.);
    }
    trail_node_ = manager->getRootSceneNode()->createChildSceneNode();
    trail_ = manager->createManualObject(id+"_furrows"); trail_->setDynamic(true);
    trail_node_->attachObject(trail_); trail_->setCastShadows(false);
    clods_ = manager->createManualObject(id+"_moving_clods"); clods_->setDynamic(true);
    trail_node_->attachObject(clods_); clods_->setCastShadows(false);
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(sdf->Get<std::string>("world_name", "hitch_farmland").first);
    state_ = transport_->Subscribe("~/hil_hitch/tillage", &TillageVisual::Activity, this);
    swaths_ = transport_->Subscribe("~/hil_hitch/worked_swaths", &TillageVisual::Swaths, this);
    update_ = gazebo::event::Events::ConnectPreRender([this]() {Render();});
    gzmsg << "Disc harrow visual ready: two disc rows throw clods, original cage roller crumbles, persistent fine bed\n";
  }
private:
  void Activity(ConstVector3dPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = msg->x() > .5; speed_ = msg->y(); received_ = std::chrono::steady_clock::now();
    if (msg->z() < sim_time_) {reset_ = true;}
    sim_time_ = msg->z();
  }
  void Swaths(ConstPosesStampedPtr &msg)
  {
    if (msg->pose_size()%2 || msg->pose_size() > 1024) {return;}
    std::deque<std::pair<ignition::math::Pose3d, ignition::math::Pose3d>> segments;
    for (int i = 0; i < msg->pose_size(); i += 2) {
      auto a = gazebo::msgs::ConvertIgn(msg->pose(i));
      auto b = gazebo::msgs::ConvertIgn(msg->pose(i+1));
      double length = (b.Pos()-a.Pos()).Length();
      if (!std::isfinite(length) || length < .05 || length > 1.) {return;}
      segments.emplace_back(a, b);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_swaths_ && segments.size() == incoming_.size() && (segments.empty() ||
        (segments.back().second.Pos()-incoming_.back().second.Pos()).Length() < 1e-9)) {return;}
    incoming_ = std::move(segments); dirty_ = true; have_swaths_ = true;
  }
  void Vertex(const ignition::math::Vector3d &point, double height,
              const ignition::math::Vector3d &normal, float shade, float alpha = 1.)
  {
    trail_->position(point.X(), point.Y(), ground_z_+height);
    trail_->normal(normal.X(), normal.Y(), normal.Z());
    trail_->textureCoord((point.X()+43.)*.8, (point.Y()+22.)*.8);
    trail_->colour(Ogre::ColourValue(shade, shade*.91f, shade*.80f, alpha));
  }
  struct Clod {
    ignition::math::Vector3d origin, velocity;
    double born, radius, seed;
    bool crushed;
  };
  ignition::math::Vector3d ClodPosition(const Clod &clod, double now) const
  {
    const double age = std::max(0., now-clod.born);
    const double flight = 2.*clod.velocity.Z()/9.81;
    const double t = std::min(age, flight);
    auto point = clod.origin+clod.velocity*t;
    point.Z() = ground_z_+clod.radius*.72+std::max(0., clod.velocity.Z()*t-4.905*t*t);
    return point;
  }
  void DrawClod(const Clod &clod, const ignition::math::Vector3d &centre, double now, unsigned &vertex)
  {
    const double age = now-clod.born;
    const double flight = 2.*clod.velocity.Z()/9.81;
    const double spin = std::min(age, flight)*(4.+clod.seed*8.);
    const ignition::math::Quaterniond rotation(spin, spin*.6, clod.seed*IGN_PI*2.);
    ignition::math::Vector3d points[8];
    for (unsigned i = 0; i < 6; ++i) {
      const double angle = i*IGN_PI/3.;
      const double r = clod.radius*(.8+.35*harrow_noise(i+clod.seed, 4.));
      points[i] = centre+rotation.RotateVector({r*std::cos(angle), r*.85*std::sin(angle), 0.});
    }
    points[6] = centre+rotation.RotateVector({clod.radius*.12, 0., clod.radius*.72});
    points[7] = centre+rotation.RotateVector({-clod.radius*.1, 0., -clod.radius*.65});
    const float alpha = clod.crushed ? std::min(1., std::max(0., 4.-age)) : 1.;
    for (unsigned i = 0; i < 6; ++i) {
      const unsigned next = (i+1)%6;
      for (unsigned tip : {6u, 7u}) {
        const auto a = points[i], b = points[next], c = points[tip];
        auto normal = (b-a).Cross(c-a).Normalized();
        if (tip == 7) {normal = -normal;}
        const float shade = .68f+.22f*harrow_noise(i+clod.seed, tip);
        for (const auto &p : {a, b, c}) {
          clods_->position(p.X(), p.Y(), std::max(ground_z_+.002, p.Z()));
          clods_->normal(normal.X(), normal.Y(), normal.Z());
          clods_->textureCoord(p.X()*3., p.Y()*3.);
          clods_->colour(Ogre::ColourValue(shade, shade, shade, alpha));
        }
        clods_->triangle(vertex, vertex+1, vertex+2); vertex += 3;
      }
    }
  }
  void AnimateSoil(bool active, double speed, double now, bool reset)
  {
    if (reset || now < last_animation_) {moving_.clear(); emission_ = 0.;}
    double dt = std::clamp(now-last_animation_, 0., .1);
    if (!reset && dt < .035) {return;}
    last_animation_ = now;
    const auto pose = visual_->WorldPose();
    const auto roller = pose.CoordPositionAdd(roller_origin_);
    const ignition::math::Quaterniond yaw(0., 0., pose.Rot().Yaw());
    std::vector<Clod> fragments;
    moving_.erase(std::remove_if(moving_.begin(), moving_.end(), [&](const Clod &clod) {
      if (now-clod.born > (clod.crushed ? 4. : 22.)) {return true;}
      const auto point = ClodPosition(clod, now);
      const auto local = yaw.RotateVectorReverse(point-roller);
      if (active && !clod.crushed && now-clod.born > .08 &&
          harrow_roller_crushes(local.X(), local.Y(), width_*.49)) {
        for (unsigned n = 0; n < 4; ++n) {
          const double seed = harrow_noise(clod.seed+n, now);
          auto velocity = yaw.RotateVector({-.12-.18*seed, (seed-.5)*.55, .25+.30*seed});
          fragments.push_back({point, velocity, now, clod.radius*(.32+.15*seed), seed, true});
        }
        return true;
      }
      return false;
    }), moving_.end());
    if (active) {
      // Distance-based emission keeps soil production tied to actual travel.
      emission_ += dt*speed*32.;
      unsigned bursts = std::min(3u, static_cast<unsigned>(emission_)); emission_ -= bursts;
      for (unsigned burst = 0; burst < bursts; ++burst) {
        for (unsigned i = 0; i < disc_origins_.size(); ++i) {
          const auto origin = pose.CoordPositionAdd(disc_origins_[i]);
          const double seed = harrow_noise(i+now*3.7, origin.Y());
          const bool front = disc_origins_[i].X() > disc_mean_x_;
          auto point = origin+yaw.RotateVector({.04*(seed-.5), .10*(seed-.5), 0.});
          point.Z() = ground_z_;
          const double side = front ? 1. : -1.;
          auto velocity = yaw.RotateVector({-.12-.28*seed, side*(.22+.4*seed),
                                             harrow_throw_speed(speed)*(.8+.45*seed)});
          moving_.push_back({point, velocity, now, (front ? .037 : .027)+.045*seed, seed, false});
        }
      }
    }
    moving_.insert(moving_.end(), fragments.begin(), fragments.end());
    if (moving_.size() > 1800) {moving_.erase(moving_.begin(), moving_.begin()+(moving_.size()-1800));}
    clods_->clear();
    if (moving_.empty()) {return;}
    clods_->begin(soil_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    unsigned vertex = 0;
    for (const auto &clod : moving_) {DrawClod(clod, ClodPosition(clod, now), now, vertex);}
    clods_->end();
  }
  void Render()
  {
    bool active, rebuild, reset;
    double speed, now;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = active_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() < .4;
      rebuild = dirty_ || reset_; dirty_ = false; speed = speed_; now = sim_time_; reset = reset_;
      if (reset_) {segments_.clear(); reset_ = false;}
      else if (rebuild) {segments_ = incoming_;}
    }
    for (unsigned i = 0; i < dust_->getNumEmitters(); ++i) {
      dust_->getEmitter(i)->setEmissionRate(active ? std::min(14., 18.*speed) : 0.);
    }
    AnimateSoil(active, speed, now, reset);
    if (!rebuild) {return;}
    trail_->clear();
    if (segments_.empty()) {return;}
    trail_->begin(soil_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    unsigned vertex = 0;
    for (const auto &segment : segments_) {
      // Disc harrows mix and crumble a shallow bed; the cage roller settles it.
      // Avoid the continuous tall ridges of a mouldboard plough.
      for (unsigned band = 0; band < 32; ++band) {
        const unsigned corners[] = {band, band+1, band+1, band};
        for (unsigned corner = 0; corner < 4; ++corner) {
          const auto &origin = corner < 2 ? segment.first : segment.second;
          double u = corners[corner]/32.;
          double y = (u-.5)*width_;
          const double edge = 1.-harrow_edge_alpha(u);
          y += edge*.045*std::sin(origin.Pos().X()*9.+origin.Pos().Y()*7.);
          auto point = origin.CoordPositionAdd({bed_offset_, y, 0});
          double noise = harrow_noise(point.X(), point.Y());
          double height = harrow_bed_height(u, noise);
          auto normal = origin.Rot().RotateVector({.26*std::sin(point.X()*25.),
            .32*std::sin(point.Y()*32.), 1.}).Normalized();
          Vertex(point, height, normal, .60f+.17f*noise, harrow_edge_alpha(u));
        }
        trail_->triangle(vertex, vertex+1, vertex+2); trail_->triangle(vertex, vertex+2, vertex+3); vertex += 4;
      }
      const double length = (segment.second.Pos()-segment.first.Pos()).Length();
      for (unsigned clod = 0; clod < 64; ++clod) {
        const double seed = harrow_noise(segment.first.Pos().X()+clod*3.7, segment.first.Pos().Y()+clod);
        const double y = width_*.46*(-1.+2.*harrow_noise(clod+segment.first.Pos().X(), segment.first.Pos().Y()+8.));
        auto centre = segment.first.CoordPositionAdd({length*seed+bed_offset_, y, 0});
        double radius = .025+.033*seed, height = harrow_clod_height(harrow_noise(centre.Y(), centre.X()));
        auto peak = centre+ignition::math::Vector3d(radius*.2, -radius*.15, height);
        for (unsigned face = 0; face < 6; ++face) {
          double a = face*IGN_PI/3., b = (face+1)*IGN_PI/3.;
          auto first = centre+ignition::math::Vector3d(radius*std::cos(a), radius*.75*std::sin(a), 0);
          auto second = centre+ignition::math::Vector3d(radius*std::cos(b), radius*.75*std::sin(b), 0);
          auto normal = (second-first).Cross(peak-first).Normalized();
          float shade = .57f+.22f*harrow_noise(face+centre.X(), centre.Y());
          Vertex(first, .010, normal, shade); Vertex(second, .010, normal, shade);
          Vertex(peak, .010+height, normal, shade);
          trail_->triangle(vertex, vertex+1, vertex+2); vertex += 3;
        }
      }
    }
    trail_->end();
    if (!logged_swath_) {gzmsg << "Disc harrow: rendered actual worked soil history\n"; logged_swath_ = true;}
  }
  gazebo::rendering::VisualPtr visual_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::SubscriberPtr state_, swaths_;
  gazebo::event::ConnectionPtr update_;
  Ogre::ParticleSystem *dust_{nullptr};
  Ogre::ManualObject *trail_{nullptr}, *clods_{nullptr};
  Ogre::SceneNode *dust_node_{nullptr}, *trail_node_{nullptr};
  Ogre::MaterialPtr dust_material_, soil_material_;
  Ogre::TexturePtr texture_;
  std::mutex mutex_;
  bool active_{false}, reset_{false}, dirty_{false}, have_swaths_{false}, logged_swath_{false};
  double speed_{0.}, sim_time_{0.}, ground_z_{-.002};
  double width_{2.54}, bed_offset_{-1.4}, disc_mean_x_{0.}, last_animation_{-1.}, emission_{0.};
  ignition::math::Vector3d roller_origin_;
  std::vector<ignition::math::Vector3d> disc_origins_;
  std::vector<Clod> moving_;
  std::deque<std::pair<ignition::math::Pose3d, ignition::math::Pose3d>> segments_, incoming_;
  std::chrono::steady_clock::time_point received_{};
};
GZ_REGISTER_VISUAL_PLUGIN(TillageVisual)
}
