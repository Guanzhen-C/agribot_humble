#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace agribot_hil
{
// Transit restraint only. Original prismatic limits and contact compliance
// are restored before any picking trajectory is allowed to execute.
class GripperHold : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    for (size_t i = 0; i < joints_.size(); ++i) {
      joints_[i] = model->GetJoint("manipulator_finger_joint" + std::to_string(i + 1));
      if (!joints_[i]) {throw std::runtime_error("HIL finger joint not found");}
      lower_[i] = joints_[i]->LowerLimit(0);
      upper_[i] = joints_[i]->UpperLimit(0);
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
        throw std::runtime_error("Invalid original finger limits");
      }
    }
    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    status_ = node_->create_publisher<std_msgs::msg::Bool>("/hil/gripper/transit_hold/status", qos);
    command_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/hil/gripper/transit_hold", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {requested_.store(msg->data);});
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & info) {Update(info);});
  }

private:
  void Update(const gazebo::common::UpdateInfo & info)
  {
    const bool hold = requested_.load();
    if (!initialized_ || hold != enabled_) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i]->SetLowerLimit(0, hold ? upper_[i] : lower_[i]);
        joints_[i]->SetUpperLimit(0, upper_[i]);
      }
      enabled_ = hold;
      initialized_ = true;
    }
    if (enabled_) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i]->SetPosition(0, upper_[i], true);
        joints_[i]->SetVelocity(0, 0.);
      }
    }
    const double now = info.simTime.Double();
    if (now < last_status_ || now - last_status_ >= .1) {
      std_msgs::msg::Bool msg;
      msg.data = enabled_;
      status_->publish(msg);
      last_status_ = now;
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  std::array<gazebo::physics::JointPtr, 2> joints_;
  std::array<double, 2> lower_{}, upper_{};
  std::atomic<bool> requested_{true};
  bool initialized_{false}, enabled_{false};
  double last_status_{-1.};
  gazebo::event::ConnectionPtr update_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_;
};
GZ_REGISTER_MODEL_PLUGIN(GripperHold)
}  // namespace agribot_hil
