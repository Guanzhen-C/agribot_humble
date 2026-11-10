#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
    const auto kind = sdf->HasElement("implement_kind") ? sdf->Get<std::string>("implement_kind") : "sprayer";
    if (kind != "sprayer" && kind != "shredder") {throw std::runtime_error("Unknown HIL implement");}
    shredder_ = kind == "shredder";
    count_ = shredder_ ? 1 : 2;
    for (size_t i = 0; i < count_; ++i) {
      joints_[i] = model->GetJoint(shredder_ ? "work_shredder_joint" :
        i == 0 ? "work_fan_joint" : "work_pump_joint");
      if (!joints_[i]) {throw std::runtime_error("Missing HIL implement rotor joint");}
    }
    command_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hil/work/target_rpm", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_.fill(0.);
        if (msg->data.size() != count_) {return;}
        for (size_t i = 0; i < count_; ++i) {
          if (!std::isfinite(msg->data[i]) || msg->data[i] < 0. ||
            msg->data[i] > (shredder_ ? 3600. : i == 0 ? 2400. : 1600.)) {target_.fill(0.); return;}
          target_[i] = msg->data[i] * 0.10471975511965977;
        }
        received_ = std::chrono::steady_clock::now();
      });
    state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/hil/work/joint_states", 5);
    if (shredder_) {
      // Merge this distinct joint with the wheel-state publisher in robot_state_publisher.
      robot_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 5);
    }
    if (!shredder_) {
      transport_.reset(new gazebo::transport::Node());
      transport_->Init(model->GetWorld()->Name());
      spray_ = transport_->Advertise<gazebo::msgs::Vector3d>("~/hil/work/spray");
    }
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
    for (size_t i = 0; i < count_; ++i) {
      // Bounded torque and real physics integration, never SetVelocity/SetPosition.
      joints_[i]->SetForce(0, std::clamp(.02 * (targets[i] - joints_[i]->GetVelocity(0)), -.5, .5));
    }
    const double now = info.simTime.Double();
    if (now < last_state_ || now - last_state_ >= .05) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.name = shredder_ ? std::vector<std::string>{"work_shredder_joint"} :
        std::vector<std::string>{"work_fan_joint", "work_pump_joint"};
      for (size_t i = 0; i < count_; ++i) {
        msg.position.push_back(joints_[i]->Position(0));
        msg.velocity.push_back(joints_[i]->GetVelocity(0));
      }
      state_->publish(msg);
      if (robot_state_) {robot_state_->publish(msg);}
      if (!shredder_) {
        gazebo::msgs::Vector3d spray;
        spray.set_x(std::clamp(std::abs(msg.velocity[1]) / 167.55, 0., 1.));
        spray.set_y(std::clamp(std::abs(msg.velocity[0]) / 251.33, 0., 1.));
        spray.set_z(now);
        spray_->Publish(spray);
      }
      last_state_ = now;
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  std::array<gazebo::physics::JointPtr, 2> joints_;
  std::array<double, 2> target_{};
  bool shredder_{false};
  size_t count_{2};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double last_state_{-1.};
  gazebo::event::ConnectionPtr update_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr spray_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_state_;
};
GZ_REGISTER_MODEL_PLUGIN(WorkMotors)
}  // namespace agribot_hil
