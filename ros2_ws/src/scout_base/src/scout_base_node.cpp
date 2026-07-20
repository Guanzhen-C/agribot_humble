#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scout_msgs/msg/scout_bms_status.hpp"
#include "scout_msgs/msg/scout_light_cmd.hpp"
#include "scout_msgs/msg/scout_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.hpp"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using westonrobot::ProtocolDetector;
using westonrobot::ScoutMiniOmniRobot;
using westonrobot::ScoutRobot;

class ScoutBaseNode : public rclcpp::Node {
 public:
  ScoutBaseNode()
      : Node("scout_base_node"),
        tf_broadcaster_(this),
        last_time_(this->now()) {
    is_scout_mini_ = declare_parameter<bool>("is_scout_mini", false);
    is_scout_omni_ = declare_parameter<bool>("is_scout_omni", false);
    agilex_joystick_ = declare_parameter<bool>("agilex_joystick", true);
    simulated_robot_ = declare_parameter<bool>("simulated_robot", false);
    pub_tf_ = declare_parameter<bool>("pub_tf", true);
    control_rate_ = declare_parameter<int>("control_rate", 50);
    port_name_ = declare_parameter<std::string>("port_name", "can0");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "");
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.8);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.65);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_name_ = declare_parameter<std::string>("odom_topic_name", "odom");

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 20);
    status_publisher_ = create_publisher<scout_msgs::msg::ScoutStatus>("/scout_status", 10);
    bms_publisher_ = create_publisher<scout_msgs::msg::ScoutBmsStatus>("/BMS_status", 10);

    if (command_timeout_sec_ <= 0.0 || max_linear_velocity_ <= 0.0 ||
        max_angular_velocity_ <= 0.0) {
      throw std::invalid_argument("Command timeout and velocity limits must be positive");
    }

    auto cmd_topic = cmd_vel_topic_.empty()
                         ? (agilex_joystick_ ? "/cmd_vel" : "/joy_teleop/cmd_vel")
                         : cmd_vel_topic_;
    motion_cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic, 10, std::bind(&ScoutBaseNode::twistCmdCallback, this, std::placeholders::_1));
    light_cmd_subscriber_ = create_subscription<scout_msgs::msg::ScoutLightCmd>(
        "/scout_light_control", 10,
        std::bind(&ScoutBaseNode::lightCmdCallback, this, std::placeholders::_1));

    setupRobot();
    last_command_time_ = now();

    const auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(control_rate_, 1)));
    timer_ = create_wall_timer(timer_period, std::bind(&ScoutBaseNode::publishLoop, this));
    RCLCPP_INFO(
        get_logger(), "Command input=%s timeout=%.3fs limits=(%.3f, %.3f)",
        cmd_topic.c_str(), command_timeout_sec_, max_linear_velocity_,
        max_angular_velocity_);
  }

  ~ScoutBaseNode() override { sendMotionCommand(geometry_msgs::msg::Twist{}); }

 private:
  void setupRobot() {
    if (simulated_robot_) {
      RCLCPP_INFO(get_logger(), "Starting scout_base_node in simulated mode");
      return;
    }

    ProtocolDetector detector;
    detector.Connect(port_name_);
    auto proto = detector.DetectProtocolVersion(5);

    if (is_scout_mini_ && is_scout_omni_) {
      if (proto == ProtocolVersion::AGX_V1) {
        robot_.reset(new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
      } else if (proto == ProtocolVersion::AGX_V2) {
        robot_.reset(new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
      } else {
        throw std::runtime_error("Unsupported Scout Mini Omni protocol");
      }
    } else {
      if (proto == ProtocolVersion::AGX_V1) {
        robot_.reset(new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini_));
      } else if (proto == ProtocolVersion::AGX_V2) {
        robot_.reset(new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini_));
      } else {
        throw std::runtime_error("Unsupported Scout protocol");
      }
    }

    robot_->Connect(port_name_);
    robot_->EnableCommandedMode();
    robot_->SetMotionCommand(0.0, 0.0);
    RCLCPP_INFO(get_logger(), "Connected to Scout over %s", port_name_.c_str());
  }

  void twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->angular.z)) {
      RCLCPP_ERROR(get_logger(), "Rejected non-finite velocity command");
      return;
    }

    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = geometry_msgs::msg::Twist{};
    current_twist_.linear.x =
        std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    current_twist_.linear.y =
        std::clamp(msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
    current_twist_.angular.z =
        std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    last_command_time_ = now();
    command_received_ = true;
  }

  geometry_msgs::msg::Twist currentCommand(const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    const double age = (stamp - last_command_time_).seconds();
    const bool fresh = command_received_ && age >= 0.0 && age <= command_timeout_sec_;
    if (!fresh) {
      if (!command_timed_out_) {
        RCLCPP_WARN(get_logger(), "Velocity command watchdog stopped the chassis");
      }
      command_timed_out_ = true;
      return geometry_msgs::msg::Twist{};
    }
    if (command_timed_out_) {
      RCLCPP_INFO(get_logger(), "Velocity command stream restored");
    }
    command_timed_out_ = false;
    return current_twist_;
  }

  void sendMotionCommand(const geometry_msgs::msg::Twist& command) noexcept {
    if (simulated_robot_ || !robot_) {
      return;
    }
    try {
      if (is_scout_mini_ && is_scout_omni_) {
        auto* omni_robot = dynamic_cast<ScoutMiniOmniRobot*>(robot_.get());
        if (omni_robot != nullptr) {
          omni_robot->SetMotionCommand(
              command.linear.x, command.angular.z, command.linear.y);
        }
      } else {
        robot_->SetMotionCommand(command.linear.x, command.angular.z);
      }
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to send chassis command: %s",
          exception.what());
    }
  }

  void lightCmdCallback(const scout_msgs::msg::ScoutLightCmd::SharedPtr msg) {
    if (simulated_robot_ || !robot_) {
      return;
    }

    if (!msg->enable_cmd_light_control) {
      robot_->DisableLightControl();
      return;
    }

    LightCommandMessage cmd;
    cmd.front_light.mode = translateLightMode(msg->front_mode);
    cmd.front_light.custom_value = msg->front_custom_value;
    cmd.rear_light.mode = translateLightMode(msg->rear_mode);
    cmd.rear_light.custom_value = msg->rear_custom_value;
    robot_->SetLightCommand(
        cmd.front_light.mode, cmd.front_light.custom_value,
        cmd.rear_light.mode, cmd.rear_light.custom_value);
  }

  AgxLightMode translateLightMode(uint8_t mode) const {
    switch (mode) {
      case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF:
        return CONST_OFF;
      case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_ON:
        return CONST_ON;
      case scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH:
        return BREATH;
      case scout_msgs::msg::ScoutLightCmd::LIGHT_CUSTOM:
      default:
        return CUSTOM;
    }
  }

  void publishLoop() {
    const auto current_time = now();
    const auto command = currentCommand(current_time);
    sendMotionCommand(command);
    const double dt = std::max((current_time - last_time_).seconds(), 0.0);
    if (!initialized_) {
      last_time_ = current_time;
      initialized_ = true;
      return;
    }

    double linear = 0.0;
    double angular = 0.0;
    if (simulated_robot_ || !robot_) {
      linear = command.linear.x;
      angular = command.angular.z;
      publishSimStatus(current_time, linear, angular);
    } else {
      const auto robot_state = robot_->GetRobotState();
      const auto actuator_state = robot_->GetActuatorState();
      linear = robot_state.motion_state.linear_velocity;
      angular = robot_state.motion_state.angular_velocity;
      publishRobotStatus(current_time, robot_state, actuator_state);
    }

    publishOdometry(current_time, linear, angular, dt);
    last_time_ = current_time;
  }

  void publishSimStatus(const rclcpp::Time& stamp, double linear, double angular) {
    scout_msgs::msg::ScoutStatus status;
    status.stamp = stamp;
    status.linear_velocity = linear;
    status.angular_velocity = angular;
    status.base_state = 0;
    status.control_mode = 1;
    status.fault_code = 0;
    status.battery_voltage = 29.5;
    status.light_control_enabled = false;
    status_publisher_->publish(status);

    scout_msgs::msg::ScoutBmsStatus bms_status;
    bms_publisher_->publish(bms_status);
  }

  template <typename RobotStateT, typename ActuatorStateT>
  void publishRobotStatus(
      const rclcpp::Time& stamp,
      const RobotStateT& robot_state,
      const ActuatorStateT& actuator_state) {
    scout_msgs::msg::ScoutStatus status;
    status.stamp = stamp;
    status.linear_velocity = robot_state.motion_state.linear_velocity;
    status.angular_velocity = robot_state.motion_state.angular_velocity;
    status.base_state = robot_state.system_state.vehicle_state;
    status.control_mode = robot_state.system_state.control_mode;
    status.fault_code = robot_state.system_state.error_code;
    status.battery_voltage = robot_state.system_state.battery_voltage;
    status.light_control_enabled = robot_state.light_state.enable_cmd_ctrl;
    status.front_light_state.mode = robot_state.light_state.front_light.mode;
    status.front_light_state.custom_value = robot_state.light_state.front_light.custom_value;
    status.rear_light_state.mode = robot_state.light_state.rear_light.mode;
    status.rear_light_state.custom_value = robot_state.light_state.rear_light.custom_value;

    if (robot_->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
      for (size_t i = 0; i < status.motor_states.size(); ++i) {
        status.motor_states[i].current = actuator_state.actuator_state[i].current;
        status.motor_states[i].rpm = actuator_state.actuator_state[i].rpm;
        status.motor_states[i].temperature = actuator_state.actuator_state[i].motor_temp;
        status.driver_states[i].driver_temperature = actuator_state.actuator_state[i].driver_temp;
      }
    } else {
      for (size_t i = 0; i < status.motor_states.size(); ++i) {
        status.motor_states[i].current = actuator_state.actuator_hs_state[i].current;
        status.motor_states[i].rpm = actuator_state.actuator_hs_state[i].rpm;
        status.motor_states[i].temperature = actuator_state.actuator_ls_state[i].motor_temp;
        status.motor_states[i].motor_pose = actuator_state.actuator_hs_state[i].pulse_count;
        status.driver_states[i].driver_state = actuator_state.actuator_ls_state[i].driver_state;
        status.driver_states[i].driver_voltage = actuator_state.actuator_ls_state[i].driver_voltage;
        status.driver_states[i].driver_temperature = actuator_state.actuator_ls_state[i].driver_temp;
      }
    }

    status_publisher_->publish(status);
    scout_msgs::msg::ScoutBmsStatus bms_status;
    bms_publisher_->publish(bms_status);
  }

  void publishOdometry(const rclcpp::Time& stamp, double linear, double angular, double dt) {
    linear_speed_ = linear;
    angular_speed_ = angular;
    position_x_ += linear_speed_ * std::cos(theta_) * dt;
    position_y_ += linear_speed_ * std::sin(theta_) * dt;
    theta_ += angular_speed_ * dt;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    if (pub_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = position_x_;
      tf_msg.transform.translation.y = position_y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = odom_quat;
      tf_broadcaster_.sendTransform(tf_msg);
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.orientation = odom_quat;
    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.angular.z = angular_speed_;
    odom_publisher_->publish(odom_msg);
  }

  bool is_scout_mini_{false};
  bool is_scout_omni_{false};
  bool agilex_joystick_{true};
  bool simulated_robot_{false};
  bool pub_tf_{true};
  bool initialized_{false};
  bool command_received_{false};
  bool command_timed_out_{false};
  int control_rate_{50};
  double command_timeout_sec_{0.25};
  double max_linear_velocity_{0.8};
  double max_angular_velocity_{0.65};
  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double theta_{0.0};
  std::string port_name_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  std::mutex twist_mutex_;
  rclcpp::Time last_time_;
  rclcpp::Time last_command_time_;
  geometry_msgs::msg::Twist current_twist_;
  std::unique_ptr<ScoutRobot> robot_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<scout_msgs::msg::ScoutBmsStatus>::SharedPtr bms_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
  rclcpp::Subscription<scout_msgs::msg::ScoutLightCmd>::SharedPtr light_cmd_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ScoutBaseNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    fprintf(stderr, "scout_base_node failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
