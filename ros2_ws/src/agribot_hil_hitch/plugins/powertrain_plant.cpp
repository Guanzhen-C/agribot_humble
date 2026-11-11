#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
extern "C" {
#include "powertrain_plant.h"
}
namespace agribot_hitch {
class PowertrainPlant : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model; base_ = model->GetLink("base_link");
    left_ = model->GetJoint("rear_left_wheel_joint"); right_ = model->GetJoint("rear_right_wheel_joint");
    if (!base_ || !left_ || !right_) {throw std::runtime_error("Missing nominal tractor driveline");}
    auto drive = model->GetSDF()->GetElement("plugin");
    for (; drive; drive = drive->GetNextElement("plugin")) {
      if (drive->Get<std::string>("name") == "ackermann_drive" &&
          drive->Get<double>("wheel_torque") != 0.) {throw std::runtime_error("Velocity motor bypass must be disabled");}
    }
    node_ = gazebo_ros::Node::Get(sdf); pp_init(&plant_); command_.ratio = 640; command_.brake = true;
    input_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/hil_hitch/actuator/powertrain", 1,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 9 || !std::all_of(msg->data.begin(), msg->data.end(), [](double v) {return std::isfinite(v);})) {return;}
        const auto &d = msg->data;
        if (d[0] < 0 || d[0] > 1 || d[1] < 0 || d[1] > 1 || d[2] < 0 || d[2] > 1 ||
            d[3] < 200 || d[3] > 900 || d[4] < 0 || d[4] > 4 || d[5] < 0 || d[5] > 4 || d[6] < 0 || d[6] > 4) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = {float(d[0]), float(d[1]), float(d[2]), float(d[3]), unsigned(d[4]), unsigned(d[5]), unsigned(d[6]), d[7] > .5, d[8] > .5};
        received_ = std::chrono::steady_clock::now();
      });
    load_ = node_->create_subscription<std_msgs::msg::Float64>("/hil_hitch/powertrain/load_force", 1,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (std::isfinite(msg->data) && msg->data >= 0 && msg->data <= 3000) {
          std::lock_guard<std::mutex> lock(mutex_); load_force_ = msg->data;
        }
      });
    output_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hil_hitch/powertrain/plant_state", rclcpp::QoS(1).best_effort());
    connection_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &info) {Update(info);});
    RCLCPP_INFO(node_->get_logger(), "Nominal powertrain FORCE plant: existing rear velocity motors fmax=0; no speed override");
  }
private:
  void Update(const gazebo::common::UpdateInfo &info)
  {
    const double now = info.simTime.Double(), dt = now-last_time_;
    if (dt <= 0 || dt > .05) {last_time_ = now; return;}
    last_time_ = now;
    pt_actuation command; float load;
    {
      std::lock_guard<std::mutex> lock(mutex_); command = command_; load = load_force_;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() > .2) {
        command = {}; command.ratio = 640; command.brake = true;
      }
    }
    const double speed = base_->WorldPose().Rot().RotateVectorReverse(base_->WorldLinearVel()).X();
    const double omega = (left_->GetVelocity(0)+right_->GetVelocity(0))*.5;
    pp_step(&plant_, &command, omega, speed, load, dt);
    for (const auto &joint : {left_, right_}) {
      // A zero-velocity bounded brake is a mechanical actuator, not a speed setpoint.
      joint->SetParam("fmax", 0, plant_.braking ? 2200. : 0.);
      if (plant_.braking) {joint->SetParam("vel", 0, 0.);}
      joint->SetForce(0, plant_.wheel_torque*.5);
    }
    base_->AddRelativeForce({-(180.+load)*std::tanh(speed/.025), 0, 0});
    if (now-last_publish_ >= .05) {
      std_msgs::msg::Float64MultiArray state;
      state.data = {double(plant_.mode), plant_.engine_omega*9.5492966, plant_.input_rpm, plant_.output_rpm,
        plant_.ratio, plant_.pressure_a, plant_.pressure_b, plant_.throttle, plant_.input_torque,
        plant_.wheel_torque, speed, double(command.gear_a), double(command.gear_b), double(plant_.gear),
        load, double(plant_.braking), plant_.engine_omega*9.5492966-plant_.input_rpm,
        now, command.ratio, double(plant_.enabled)};
      output_->publish(state); last_publish_ = now;
    }
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo::physics::JointPtr left_, right_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr load_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_;
  pt_plant plant_{}; pt_actuation command_{};
  std::mutex mutex_; float load_force_{0};
  double last_time_{0}, last_publish_{0};
  std::chrono::steady_clock::time_point received_{};
};
GZ_REGISTER_MODEL_PLUGIN(PowertrainPlant)
}
