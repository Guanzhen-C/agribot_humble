#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace agribot_hil
{
class WorkMotors : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    for (size_t i = 0; i < 2; ++i) {
      joints_[i] = model->GetJoint(i == 0 ? "work_fan_joint" : "work_pump_joint");
      if (!joints_[i]) {throw std::runtime_error("Missing HIL implement rotor joint");}
    }
    command_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hil/work/target_rpm", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_.fill(0.);
        if (msg->data.size() != 2) {return;}
        for (size_t i = 0; i < 2; ++i) {
          if (!std::isfinite(msg->data[i]) || msg->data[i] < 0. ||
            msg->data[i] > (i == 0 ? 2400. : 1600.)) {target_.fill(0.); return;}
          target_[i] = msg->data[i] * 0.10471975511965977;
        }
        received_ = std::chrono::steady_clock::now();
      });
    state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/hil/work/joint_states", 5);
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(model->GetWorld()->Name());
    spray_ = transport_->Advertise<gazebo::msgs::Vector3d>("~/hil/work/spray");
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & info) {Update(info);});
  }

private:
  void Update(const gazebo::common::UpdateInfo & info)
  {
    std::array<double, 2> targets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      targets = target_;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() > .3) {
        targets.fill(0.);
      }
    }
    for (size_t i = 0; i < 2; ++i) {
      // Bounded torque and real physics integration, never SetVelocity/SetPosition.
      joints_[i]->SetForce(0, std::clamp(.02 * (targets[i] - joints_[i]->GetVelocity(0)), -.5, .5));
    }
    const double now = info.simTime.Double();
    if (now < last_state_ || now - last_state_ >= .05) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.name = {"work_fan_joint", "work_pump_joint"};
      for (auto & joint : joints_) {
        msg.position.push_back(joint->Position(0));
        msg.velocity.push_back(joint->GetVelocity(0));
      }
      state_->publish(msg);
      gazebo::msgs::Vector3d spray;
      spray.set_x(std::clamp(std::abs(msg.velocity[1]) / 167.55, 0., 1.));
      spray.set_y(std::clamp(std::abs(msg.velocity[0]) / 251.33, 0., 1.));
      spray.set_z(now);
      spray_->Publish(spray);
      last_state_ = now;
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  std::array<gazebo::physics::JointPtr, 2> joints_;
  std::array<double, 2> target_{};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double last_state_{-1.};
  gazebo::event::ConnectionPtr update_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr spray_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_;
};
GZ_REGISTER_MODEL_PLUGIN(WorkMotors)
}  // namespace agribot_hil
