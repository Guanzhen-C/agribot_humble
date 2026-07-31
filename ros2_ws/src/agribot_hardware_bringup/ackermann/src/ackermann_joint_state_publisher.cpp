#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace agribot_hardware_bringup
{
namespace
{

constexpr std::size_t kFrontLeftSteering = 0;
constexpr std::size_t kFrontRightSteering = 1;
constexpr std::size_t kFrontLeftWheel = 2;
constexpr std::size_t kFrontRightWheel = 3;
constexpr std::size_t kRearLeftWheel = 4;
constexpr std::size_t kRearRightWheel = 5;
constexpr double kTwoPi = 6.28318530717958647692;

double require_positive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
  return value;
}

}  // namespace

class AckermannJointStatePublisher : public rclcpp::Node
{
public:
  AckermannJointStatePublisher()
  : Node("ackermann_joint_state_publisher")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    joint_states_topic_ =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    wheelbase_m_ = require_positive(
      declare_parameter<double>("wheelbase_m", 0.5265855),
        "wheelbase_m");
    front_track_m_ = require_positive(
      declare_parameter<double>("front_track_m", 0.589931), "front_track_m");
    rear_track_m_ = require_positive(
      declare_parameter<double>("rear_track_m", 0.590517), "rear_track_m");
    wheel_radius_m_ = require_positive(
      declare_parameter<double>("wheel_radius_m", 0.1275), "wheel_radius_m");
    max_steering_angle_rad_ = require_positive(
      declare_parameter<double>("max_steering_angle_rad", 0.384),
      "max_steering_angle_rad");
    min_linear_speed_mps_ = require_positive(
      declare_parameter<double>("min_linear_speed_mps", 0.02),
      "min_linear_speed_mps");
    feedback_timeout_sec_ = require_positive(
      declare_parameter<double>("feedback_timeout_sec", 0.6),
      "feedback_timeout_sec");
    const double publish_rate_hz = require_positive(
      declare_parameter<double>("publish_rate_hz", 30.0), "publish_rate_hz");

    joint_names_ = {
      "front_left_steering_joint",
      "front_right_steering_joint",
      "front_left_wheel_joint",
      "front_right_wheel_joint",
      "rear_left_wheel_joint",
      "rear_right_wheel_joint",
    };

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&AckermannJointStatePublisher::handle_odometry, this,
        std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AckermannJointStatePublisher::publish_joint_states, this));
  }

private:
  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    linear_velocity_mps_ = message->twist.twist.linear.x;
    angular_velocity_rad_s_ = message->twist.twist.angular.z;
    last_feedback_time_ = now();
    have_feedback_ = true;

    if (std::abs(linear_velocity_mps_) < min_linear_speed_mps_) {
      return;
    }

    double center_angle = std::atan(
      wheelbase_m_ * angular_velocity_rad_s_ / linear_velocity_mps_);
    center_angle = std::clamp(
      center_angle, -max_steering_angle_rad_, max_steering_angle_rad_);

    if (std::abs(center_angle) < 1e-6) {
      positions_[kFrontLeftSteering] = 0.0;
      positions_[kFrontRightSteering] = 0.0;
      return;
    }

    const double turn_radius = wheelbase_m_ / std::tan(center_angle);
    positions_[kFrontLeftSteering] = std::clamp(
      std::atan(wheelbase_m_ / (turn_radius - front_track_m_ * 0.5)),
      -max_steering_angle_rad_, max_steering_angle_rad_);
    positions_[kFrontRightSteering] = std::clamp(
      std::atan(wheelbase_m_ / (turn_radius + front_track_m_ * 0.5)),
      -max_steering_angle_rad_, max_steering_angle_rad_);
  }

  std::array<double, 4> wheel_angular_velocities(const rclcpp::Time & stamp) const
  {
    if (!have_feedback_ || (stamp - last_feedback_time_).seconds() > feedback_timeout_sec_) {
      return {0.0, 0.0, 0.0, 0.0};
    }

    const double rear_left =
      linear_velocity_mps_ - angular_velocity_rad_s_ * rear_track_m_ * 0.5;
    const double rear_right =
      linear_velocity_mps_ + angular_velocity_rad_s_ * rear_track_m_ * 0.5;

    double front_left = 0.0;
    double front_right = 0.0;
    if (std::abs(linear_velocity_mps_) >= min_linear_speed_mps_) {
      const double front_lateral = angular_velocity_rad_s_ * wheelbase_m_;
      front_left = std::copysign(
        std::hypot(
          linear_velocity_mps_ - angular_velocity_rad_s_ * front_track_m_ * 0.5,
          front_lateral),
        linear_velocity_mps_);
      front_right = std::copysign(
        std::hypot(
          linear_velocity_mps_ + angular_velocity_rad_s_ * front_track_m_ * 0.5,
          front_lateral),
        linear_velocity_mps_);
    }

    return {
      front_left / wheel_radius_m_,
      front_right / wheel_radius_m_,
      rear_left / wheel_radius_m_,
      rear_right / wheel_radius_m_,
    };
  }

  void publish_joint_states()
  {
    const rclcpp::Time stamp = now();
    const auto wheel_velocities = wheel_angular_velocities(stamp);

    if (have_publish_time_) {
      const double dt = (stamp - last_publish_time_).seconds();
      if (dt > 0.0 && dt <= 0.25) {
        positions_[kFrontLeftWheel] = std::remainder(
          positions_[kFrontLeftWheel] + wheel_velocities[0] * dt, kTwoPi);
        positions_[kFrontRightWheel] = std::remainder(
          positions_[kFrontRightWheel] + wheel_velocities[1] * dt, kTwoPi);
        positions_[kRearLeftWheel] = std::remainder(
          positions_[kRearLeftWheel] + wheel_velocities[2] * dt, kTwoPi);
        positions_[kRearRightWheel] = std::remainder(
          positions_[kRearRightWheel] + wheel_velocities[3] * dt, kTwoPi);
      }
    }
    last_publish_time_ = stamp;
    have_publish_time_ = true;

    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name.assign(joint_names_.begin(), joint_names_.end());
    message.position.assign(positions_.begin(), positions_.end());
    message.velocity = {
      0.0, 0.0,
      wheel_velocities[0], wheel_velocities[1],
      wheel_velocities[2], wheel_velocities[3],
    };
    publisher_->publish(message);
  }

  std::string odom_topic_;
  std::string joint_states_topic_;
  double wheelbase_m_{0.5265855};
  double front_track_m_{0.589931};
  double rear_track_m_{0.590517};
  double wheel_radius_m_{0.1275};
  double max_steering_angle_rad_{0.384};
  double min_linear_speed_mps_{0.02};
  double feedback_timeout_sec_{0.6};
  double linear_velocity_mps_{0.0};
  double angular_velocity_rad_s_{0.0};
  bool have_feedback_{false};
  bool have_publish_time_{false};
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  std::array<std::string, 6> joint_names_;
  std::array<double, 6> positions_{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<agribot_hardware_bringup::AckermannJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
