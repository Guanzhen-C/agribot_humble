#include <functional>
#include <map>
#include <stdexcept>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/plugins/WheelSlipPlugin.hh>

#include "tracked_slip_policy.hpp"

namespace agribot_vehicle_description
{
// Keep Gazebo's wheel contact model; only select compliance for straight/skid motion.
class SimulationTrackedWheelSlip : public gazebo::WheelSlipPlugin
{
public:
  ~SimulationTrackedWheelSlip() override {update_.reset();}

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    straight_ = sdf->Get<double>("straight_compliance");
    turning_slip_ = sdf->Get<double>("turning_slip");
    turn_start_ = sdf->Get<double>("turn_start_speed_difference");
    turn_full_ = sdf->Get<double>("turn_full_speed_difference");
    if (!std::isfinite(straight_) || straight_ < 0 ||
      !std::isfinite(turning_slip_) || turning_slip_ <= 0 ||
      !std::isfinite(turn_start_) || turn_start_ < 0 ||
      !std::isfinite(turn_full_) || turn_full_ <= turn_start_)
    {
      throw std::runtime_error("Invalid tracked wheel slip policy");
    }
    for (auto wheel = sdf->GetElement("wheel"); wheel;
      wheel = wheel->GetNextElement("wheel"))
    {
      const auto name = wheel->Get<std::string>("link_name");
      const double force = wheel->Get<double>("wheel_normal_force");
      const bool left = name.find("_left_") != std::string::npos;
      if (!std::isfinite(force) || force <= 0 ||
        (!left && name.find("_right_") == std::string::npos))
      {
        throw std::runtime_error("Invalid tracked contact wheel");
      }
      wheels_[name] = {left, force};
    }
    gazebo::WheelSlipPlugin::Load(model, sdf);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SimulationTrackedWheelSlip::UpdateCompliance, this));
  }

  void Fini() override
  {
    update_.reset();
    gazebo::WheelSlipPlugin::Fini();
  }

private:
  void UpdateCompliance()
  {
    std::map<std::string, ignition::math::Vector3d> slips;
    GetSlips(slips);
    double left = 0, right = 0;
    unsigned int nl = 0, nr = 0;
    for (const auto & [name, wheel] : wheels_) {
      const auto found = slips.find(name);
      if (found == slips.end()) {continue;}
      if (wheel.left) {left += found->second.Z(); ++nl;}
      else {right += found->second.Z(); ++nr;}
    }
    if (!nl || !nr) {return;}
    left /= nl;
    right /= nr;
    for (const auto & [name, wheel] : wheels_) {
      const auto found = slips.find(name);
      if (found == slips.end()) {continue;}
      SetSlipComplianceLateral(name, tracked_lateral_compliance(
        left, right, found->second.Z(), wheel.force,
        straight_, turning_slip_, turn_start_, turn_full_));
    }
  }

  struct Wheel {bool left; double force;};
  std::map<std::string, Wheel> wheels_;
  gazebo::event::ConnectionPtr update_;
  double straight_{0.1}, turning_slip_{0.003}, turn_start_{0.02}, turn_full_{0.08};
};
GZ_REGISTER_MODEL_PLUGIN(SimulationTrackedWheelSlip)
}  // namespace agribot_vehicle_description
