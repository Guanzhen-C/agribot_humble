#include <cmath>
#include <algorithm>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

namespace agribot_vehicle_description
{
class SimulationVisualRotor : public gazebo::VisualPlugin
{
public:
  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    visual_ = visual;
    const auto base = visual_->GetParent();
    base_ = base;
    const auto name = base->Name();
    const auto scope = name.substr(0, name.rfind("::") + 2);
    source_name_ = scope + sdf->Get<std::string>("source_link");
    ratio_ = sdf->Get<double>("ratio");
    if (sdf->HasElement("path")) {
      std::istringstream input(sdf->Get<std::string>("path"));
      double x, z;
      while (input >> x >> z) {path_.emplace_back(x, z);}
      distances_.push_back(0.0);
      for (size_t i = 0; i < path_.size(); ++i) {
        const auto & a = path_[i];
        const auto & b = path_[(i + 1) % path_.size()];
        distances_.push_back(distances_.back() + std::hypot(b.X()-a.X(), b.Y()-a.Y()));
      }
      offset_ = sdf->Get<double>("distance");
      center_y_ = sdf->Get<double>("center_y");
    }
    update_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&SimulationVisualRotor::Update, this));
  }

private:
  void Update()
  {
    if (!source_) {
      source_ = visual_->GetScene()->GetVisual(source_name_);
      if (!source_) {
        if (++missing_frames_ == 120) {
          gzwarn << "Visual rotor source not present in rendering scene: " << source_name_ << "\n";
        }
        return;
      }
    }
    const auto rotation = base_->WorldPose().Rot().Inverse() * source_->WorldPose().Rot();
    const double phase = 2.0 * std::atan2(rotation.Y(), rotation.W());
    if (!initialized_) {
      total_ = phase;
      initialized_ = true;
    } else {
      // Unwrap before applying the radius ratio, including reverse motion.
      total_ += std::remainder(phase - previous_, 2.0 * M_PI);
    }
    previous_ = phase;
    if (!path_.empty()) {
      double distance = std::fmod(offset_ + total_ * ratio_, distances_.back());
      if (distance < 0) {distance += distances_.back();}
      const auto i = std::min(static_cast<size_t>(std::upper_bound(
        distances_.begin(), distances_.end(), distance)-distances_.begin()-1), path_.size()-1);
      const auto & a = path_[i];
      const auto & b = path_[(i+1) % path_.size()];
      const double fraction = (distance-distances_[i])/(distances_[i+1]-distances_[i]);
      const auto position = a + fraction * (b-a);
      visual_->SetPose(ignition::math::Pose3d(position.X(), center_y_, position.Y(),
        0, -std::atan2(b.Y()-a.Y(), b.X()-a.X()), 0));
    } else {
      visual_->SetRotation(ignition::math::Quaterniond(0, total_ * ratio_, 0));
    }
  }

  gazebo::rendering::VisualPtr visual_, base_, source_;
  gazebo::event::ConnectionPtr update_;
  std::string source_name_;
  double ratio_{1}, total_{0}, previous_{0};
  double offset_{0}, center_y_{0};
  std::vector<ignition::math::Vector2d> path_;
  std::vector<double> distances_;
  bool initialized_{false};
  unsigned int missing_frames_{0};
};
GZ_REGISTER_VISUAL_PLUGIN(SimulationVisualRotor)
}  // namespace agribot_vehicle_description
