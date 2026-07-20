#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scout_msgs/msg/scout_status.hpp"
#include "std_msgs/msg/bool.hpp"

#include "agribot_hardware_bringup/chassis_can_protocol.hpp"

namespace agribot_hardware_bringup
{
namespace protocol = chassis_can;
using namespace std::chrono_literals;

class ChassisCanNode : public rclcpp::Node
{
public:
  ChassisCanNode()
  : Node("chassis_can_node")
  {
    chassis_type_ = declare_parameter<std::string>("chassis_type", "differential");
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    command_topic_ = declare_parameter<std::string>("command_topic", "/hardware/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "wheel_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 10.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 1.2);
    require_feedback_before_motion_ =
      declare_parameter<bool>("require_feedback_before_motion", true);
    require_autonomous_mode_ = declare_parameter<bool>("require_autonomous_mode", true);
    legacy_brake_byte_ = declare_parameter<bool>("legacy_brake_byte", true);
    invert_left_motor_ = declare_parameter<bool>("invert_left_motor", false);
    invert_right_motor_ = declare_parameter<bool>("invert_right_motor", false);
    headlight_ = declare_parameter<bool>("headlight", false);

    differential_config_.track_width_m =
      declare_parameter<double>("track_width_m", 0.94);
    differential_config_.wheel_diameter_m =
      declare_parameter<double>("wheel_diameter_m", 0.20);
    differential_config_.reduction_ratio =
      declare_parameter<double>("reduction_ratio", 30.0);
    differential_config_.max_motor_rpm =
      declare_parameter<double>("max_motor_rpm", 3000.0);
    differential_config_.max_linear_velocity =
      declare_parameter<double>("max_linear_velocity", 0.80);
    differential_config_.max_angular_velocity =
      declare_parameter<double>("max_angular_velocity", 1.4);

    ackermann_config_.wheelbase_m = declare_parameter<double>("wheelbase_m", 0.65);
    ackermann_config_.max_steering_angle_rad =
      declare_parameter<double>("max_steering_angle_rad", 0.60);
    ackermann_config_.max_linear_velocity = differential_config_.max_linear_velocity;
    ackermann_config_.max_angular_velocity =
      declare_parameter<double>("ackermann_max_yaw_rate", 0.65);
    ackermann_config_.minimum_motion_speed =
      declare_parameter<double>("minimum_motion_speed", 0.02);
    ackermann_command_id_ = static_cast<uint32_t>(
      declare_parameter<int64_t>(
        "ackermann_command_id", protocol::kReferenceAckermannCommandId));
    ackermann_state_id_ = static_cast<uint32_t>(
      declare_parameter<int64_t>(
        "ackermann_state_id", protocol::kReferenceAckermannStateId));
    allow_unverified_ackermann_protocol_ =
      declare_parameter<bool>("allow_unverified_ackermann_protocol", false);

    validateParameters();
    openSocket();

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 20,
      std::bind(&ChassisCanNode::handleCommand, this, std::placeholders::_1));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    status_publisher_ = create_publisher<scout_msgs::msg::ScoutStatus>("/scout_status", 10);
    emergency_stop_publisher_ =
      create_publisher<std_msgs::msg::Bool>("/hardware/chassis_e_stop", 10);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());

    const auto send_period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(send_period),
      std::bind(&ChassisCanNode::sendCommand, this));
    receive_timer_ = create_wall_timer(5ms, std::bind(&ChassisCanNode::receiveFrames, this));
    diagnostics_timer_ = create_wall_timer(
      1s, std::bind(&ChassisCanNode::publishDiagnostics, this));

    context_ = get_node_base_interface()->get_context();
    pre_shutdown_callback_ = context_->add_pre_shutdown_callback(
      [this]() {
        RCLCPP_WARN(get_logger(), "ROS shutdown requested; transmitting CAN brake frames");
        sendStopFrames();
      });
    pre_shutdown_callback_registered_ = true;

    RCLCPP_INFO(
      get_logger(), "CAN chassis ready: type=%s interface=%s command=%s rate=%.1f Hz",
      chassis_type_.c_str(), can_interface_.c_str(), command_topic_.c_str(), send_rate_hz_);
  }

  ~ChassisCanNode() override
  {
    if (pre_shutdown_callback_registered_ && context_) {
      context_->remove_pre_shutdown_callback(pre_shutdown_callback_);
    }
    sendStopFrames();
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  void validateParameters()
  {
    if (chassis_type_ != "differential" && chassis_type_ != "ackermann") {
      throw std::invalid_argument("chassis_type must be 'differential' or 'ackermann'");
    }
    if (send_rate_hz_ <= 0.0 || command_timeout_sec_ <= 0.0 ||
      feedback_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("CAN timing parameters must be positive");
    }
    if (chassis_type_ == "ackermann" && !allow_unverified_ackermann_protocol_) {
      throw std::invalid_argument(
              "Ackermann CAN layout is a reference only; set "
              "allow_unverified_ackermann_protocol:=true only after controller confirmation");
    }
    if (ackermann_command_id_ > CAN_SFF_MASK || ackermann_state_id_ > CAN_SFF_MASK) {
      throw std::invalid_argument("only standard 11-bit CAN IDs are supported");
    }

    // Exercise the validation in the pure protocol layer during startup.
    if (chassis_type_ == "differential") {
      (void)protocol::differentialFromTwist(0.0, 0.0, differential_config_, true);
    } else {
      (void)protocol::ackermannFromTwist(0.0, 0.0, ackermann_config_, true);
    }
  }

  void openSocket()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "socket(PF_CAN)");
    }

    struct ifreq request {};
    if (can_interface_.size() >= IFNAMSIZ) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::invalid_argument("CAN interface name is too long");
    }
    std::strncpy(request.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
      const auto error = errno;
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::system_error(error, std::generic_category(), "CAN interface lookup");
    }

    std::array<struct can_filter, 3> filters{};
    std::size_t filter_count = 0;
    if (chassis_type_ == "differential") {
      filters[0] = {protocol::kDifferentialStateId, CAN_SFF_MASK};
      filters[1] = {protocol::kLeftMotorStateId, CAN_SFF_MASK};
      filters[2] = {protocol::kRightMotorStateId, CAN_SFF_MASK};
      filter_count = 3;
    } else {
      filters[0] = {ackermann_state_id_, CAN_SFF_MASK};
      filter_count = 1;
    }
    if (setsockopt(
        socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filter_count * sizeof(struct can_filter))) < 0)
    {
      const auto error = errno;
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::system_error(error, std::generic_category(), "CAN_RAW_FILTER");
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      const auto error = errno;
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::system_error(error, std::generic_category(), "bind(SocketCAN)");
    }
  }

  void handleCommand(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const std::array<double, 2> values{message->linear.x, message->angular.z};
    if (!std::isfinite(values[0]) || !std::isfinite(values[1])) {
      RCLCPP_ERROR(get_logger(), "Rejected non-finite chassis command");
      command_received_ = false;
      return;
    }
    std::lock_guard<std::mutex> guard(state_mutex_);
    latest_command_ = *message;
    last_command_time_ = now();
    command_received_ = true;
  }

  bool feedbackFresh(const rclcpp::Time & current_time) const
  {
    const auto isFresh = [&](bool received, const rclcpp::Time & stamp) {
        if (!received) {
          return false;
        }
        const double age = (current_time - stamp).seconds();
        return age >= 0.0 && age <= feedback_timeout_sec_;
      };
    if (chassis_type_ == "differential") {
      return isFresh(chassis_state_received_, chassis_state_time_) &&
             isFresh(left_motor_received_, left_motor_time_) &&
             isFresh(right_motor_received_, right_motor_time_);
    }
    return isFresh(ackermann_state_received_, ackermann_state_time_);
  }

  bool feedbackAllowsMotion(const rclcpp::Time & current_time) const
  {
    if (!require_feedback_before_motion_) {
      return true;
    }
    if (!feedbackFresh(current_time)) {
      return false;
    }
    if (chassis_type_ == "differential") {
      if (!chassis_state_.has_value() || chassis_state_->emergency_stop) {
        return false;
      }
      if (require_autonomous_mode_ && chassis_state_->work_mode != 1U) {
        return false;
      }
      if (chassis_state_->remote_comm_fault || chassis_state_->autonomy_comm_fault ||
        chassis_state_->motor_comm_fault || chassis_state_->bms_comm_fault)
      {
        return false;
      }
      if ((left_motor_state_.has_value() && left_motor_state_->hasFault()) ||
        (right_motor_state_.has_value() && right_motor_state_->hasFault()))
      {
        return false;
      }
      return true;
    }
    return ackermann_state_.has_value() && ackermann_state_->enabled &&
           !ackermann_state_->emergency_stop && !ackermann_state_->fault;
  }

  void sendCommand()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto current_time = now();
    const bool command_fresh = command_received_ &&
      (current_time - last_command_time_).seconds() >= 0.0 &&
      (current_time - last_command_time_).seconds() <= command_timeout_sec_;
    const bool motion_allowed = command_fresh && feedbackAllowsMotion(current_time);

    try {
      protocol::Frame frame;
      if (chassis_type_ == "differential") {
        auto command = protocol::differentialFromTwist(
          latest_command_.linear.x, latest_command_.angular.z,
          differential_config_, !motion_allowed, headlight_);
        if (invert_left_motor_) {
          command.left_percent *= -1.0;
        }
        if (invert_right_motor_) {
          command.right_percent *= -1.0;
        }
        frame = protocol::encodeDifferentialCommand(
          command, transmit_counter_, legacy_brake_byte_);
      } else {
        const auto command = protocol::ackermannFromTwist(
          latest_command_.linear.x, latest_command_.angular.z,
          ackermann_config_, !motion_allowed, headlight_);
        frame = protocol::encodeReferenceAckermannCommand(
          command, transmit_counter_, ackermann_command_id_);
      }
      writeFrame(frame);
      transmit_counter_ = (transmit_counter_ + 1U) & 0x0fU;
      motion_command_active_ = motion_allowed;
    } catch (const std::exception & exception) {
      ++transmit_errors_;
      motion_command_active_ = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "CAN transmit failed: %s", exception.what());
    }
  }

  void writeFrame(const protocol::Frame & frame)
  {
    struct can_frame raw_frame {};
    raw_frame.can_id = frame.id;
    raw_frame.can_dlc = protocol::kPayloadSize;
    std::copy(frame.data.begin(), frame.data.end(), raw_frame.data);
    const auto bytes = write(socket_fd_, &raw_frame, sizeof(raw_frame));
    if (bytes != static_cast<ssize_t>(sizeof(raw_frame))) {
      throw std::system_error(errno, std::generic_category(), "write(SocketCAN)");
    }
  }

  void receiveFrames()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    for (std::size_t count = 0; count < 64; ++count) {
      struct can_frame raw_frame {};
      const auto bytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));
      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      if (bytes < 0) {
        ++receive_errors_;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "CAN receive failed: %s", std::strerror(errno));
        return;
      }
      if (bytes != static_cast<ssize_t>(sizeof(raw_frame)) ||
        raw_frame.can_dlc != protocol::kPayloadSize ||
        (raw_frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U)
      {
        ++invalid_frames_;
        continue;
      }

      protocol::Frame frame;
      frame.id = raw_frame.can_id & CAN_SFF_MASK;
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), frame.data.begin());
      processFrame(frame);
    }
  }

  bool counterIsNew(const protocol::Frame & frame)
  {
    const auto counter = protocol::rollingCounter(frame.data);
    const auto found = receive_counters_.find(frame.id);
    if (found != receive_counters_.end()) {
      if (counter == found->second) {
        ++replay_frames_;
        return false;
      }
      const uint8_t expected = (found->second + 1U) & 0x0fU;
      if (counter != expected) {
        ++counter_errors_;
      }
    }
    receive_counters_[frame.id] = counter;
    return true;
  }

  void processFrame(const protocol::Frame & frame)
  {
    if (!protocol::hasValidChecksum(frame.data)) {
      ++checksum_errors_;
      return;
    }
    if (!counterIsNew(frame)) {
      return;
    }
    const auto current_time = now();

    if (chassis_type_ == "differential") {
      if (frame.id == protocol::kDifferentialStateId) {
        chassis_state_ = protocol::decodeChassisState(frame);
        if (!chassis_state_.has_value()) {
          ++invalid_frames_;
          return;
        }
        chassis_state_received_ = true;
        chassis_state_time_ = current_time;
        std_msgs::msg::Bool emergency;
        emergency.data = chassis_state_->emergency_stop;
        emergency_stop_publisher_->publish(emergency);
      } else {
        const auto motor = protocol::decodeMotorState(frame);
        if (!motor.has_value()) {
          ++invalid_frames_;
          return;
        }
        if (frame.id == protocol::kLeftMotorStateId) {
          left_motor_state_ = motor;
          left_motor_update_ = true;
          left_motor_received_ = true;
          left_motor_time_ = current_time;
        } else if (frame.id == protocol::kRightMotorStateId) {
          right_motor_state_ = motor;
          right_motor_update_ = true;
          right_motor_received_ = true;
          right_motor_time_ = current_time;
        }
        if (left_motor_update_ && right_motor_update_) {
          updateDifferentialOdometry(current_time);
          left_motor_update_ = false;
          right_motor_update_ = false;
        }
      }
    } else {
      ackermann_state_ = protocol::decodeReferenceAckermannState(frame, ackermann_state_id_);
      if (!ackermann_state_.has_value()) {
        ++invalid_frames_;
        return;
      }
      ackermann_state_received_ = true;
      ackermann_state_time_ = current_time;
      std_msgs::msg::Bool emergency;
      emergency.data = ackermann_state_->emergency_stop;
      emergency_stop_publisher_->publish(emergency);
      const double angular_velocity = std::abs(ackermann_state_->speed_mps) < 1e-6 ? 0.0 :
        ackermann_state_->speed_mps * std::tan(ackermann_state_->steering_angle_rad) /
        ackermann_config_.wheelbase_m;
      updateOdometry(current_time, ackermann_state_->speed_mps, angular_velocity);
    }

    publishStatus();
  }

  void updateDifferentialOdometry(const rclcpp::Time & stamp)
  {
    int16_t left_rpm = left_motor_state_->rpm;
    int16_t right_rpm = right_motor_state_->rpm;
    if (invert_left_motor_) {
      left_rpm = static_cast<int16_t>(-left_rpm);
    }
    if (invert_right_motor_) {
      right_rpm = static_cast<int16_t>(-right_rpm);
    }
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    protocol::motorRpmToTwist(
      left_rpm, right_rpm, differential_config_, linear_velocity, angular_velocity);
    updateOdometry(stamp, linear_velocity, angular_velocity);
  }

  void updateOdometry(
    const rclcpp::Time & stamp, double linear_velocity, double angular_velocity)
  {
    if (last_odom_time_.nanoseconds() > 0) {
      const double delta = (stamp - last_odom_time_).seconds();
      if (delta > 0.0 && delta < feedback_timeout_sec_ * 2.0) {
        x_ += linear_velocity * std::cos(yaw_) * delta;
        y_ += linear_velocity * std::sin(yaw_) * delta;
        yaw_ += angular_velocity * delta;
      }
    }
    last_odom_time_ = stamp;
    measured_linear_velocity_ = linear_velocity;
    measured_angular_velocity_ = angular_velocity;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    odometry.twist.twist.linear.x = linear_velocity;
    odometry.twist.twist.angular.z = angular_velocity;
    odometry.pose.covariance[0] = 0.05;
    odometry.pose.covariance[7] = 0.05;
    odometry.pose.covariance[35] = 0.10;
    odometry.twist.covariance[0] = 0.02;
    odometry.twist.covariance[35] = 0.05;
    odom_publisher_->publish(odometry);
  }

  void publishStatus()
  {
    scout_msgs::msg::ScoutStatus status;
    status.stamp = now();
    status.linear_velocity = measured_linear_velocity_;
    status.angular_velocity = measured_angular_velocity_;

    if (chassis_type_ == "differential" && chassis_state_.has_value()) {
      status.control_mode = chassis_state_->work_mode;
      status.battery_voltage = chassis_state_->battery_voltage;
      const bool chassis_fault = chassis_state_->emergency_stop ||
        chassis_state_->remote_comm_fault || chassis_state_->autonomy_comm_fault ||
        chassis_state_->motor_comm_fault || chassis_state_->bms_comm_fault;
      status.base_state = chassis_fault ? 1U : 0U;
      status.fault_code = chassis_fault ? 1U : 0U;

      const auto fillMotor = [&](std::size_t index, const protocol::MotorState & motor) {
          status.motor_states[index].current = motor.current;
          status.motor_states[index].rpm = motor.rpm;
          status.motor_states[index].temperature = motor.temperature_c;
          status.motor_states[index].motor_pose = 0.0;
        };
      if (left_motor_state_.has_value()) {
        fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_LEFT, *left_motor_state_);
        fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_REAR_LEFT, *left_motor_state_);
        if (left_motor_state_->hasFault()) {
          status.fault_code |= 0x02U;
        }
      }
      if (right_motor_state_.has_value()) {
        fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_RIGHT, *right_motor_state_);
        fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_REAR_RIGHT, *right_motor_state_);
        if (right_motor_state_->hasFault()) {
          status.fault_code |= 0x04U;
        }
      }
    } else if (ackermann_state_.has_value()) {
      status.control_mode = ackermann_state_->enabled ? 1U : 0U;
      status.battery_voltage = ackermann_state_->battery_voltage;
      status.base_state =
        (ackermann_state_->emergency_stop || ackermann_state_->fault) ? 1U : 0U;
      status.fault_code = ackermann_state_->fault ? 1U : 0U;
    }
    status_publisher_->publish(status);
  }

  static diagnostic_msgs::msg::KeyValue keyValue(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
  }

  void publishDiagnostics()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agribot/chassis_can";
    status.hardware_id = can_interface_;

    const bool fresh = feedbackFresh(now());
    if (!fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "chassis feedback missing or stale";
    } else if (!feedbackAllowsMotion(now())) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "feedback received but motion is inhibited";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "CAN protocol and feedback healthy";
    }
    status.values.push_back(keyValue("chassis_type", chassis_type_));
    status.values.push_back(keyValue("feedback_fresh", fresh ? "true" : "false"));
    status.values.push_back(
      keyValue("command_active", motion_command_active_ ? "true" : "false"));
    status.values.push_back(keyValue("checksum_errors", std::to_string(checksum_errors_)));
    status.values.push_back(keyValue("counter_errors", std::to_string(counter_errors_)));
    status.values.push_back(keyValue("replay_frames", std::to_string(replay_frames_)));
    status.values.push_back(keyValue("invalid_frames", std::to_string(invalid_frames_)));
    status.values.push_back(keyValue("transmit_errors", std::to_string(transmit_errors_)));
    status.values.push_back(keyValue("receive_errors", std::to_string(receive_errors_)));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  void sendStopFrames() noexcept
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    if (socket_fd_ < 0 || stop_frames_sent_) {
      return;
    }
    stop_frames_sent_ = true;
    try {
      for (int count = 0; count < 3; ++count) {
        protocol::Frame frame;
        if (chassis_type_ == "differential") {
          protocol::DifferentialCommand command;
          command.brake = true;
          frame = protocol::encodeDifferentialCommand(
            command, transmit_counter_, legacy_brake_byte_);
        } else {
          protocol::AckermannCommand command;
          command.brake = true;
          frame = protocol::encodeReferenceAckermannCommand(
            command, transmit_counter_, ackermann_command_id_);
        }
        writeFrame(frame);
        transmit_counter_ = (transmit_counter_ + 1U) & 0x0fU;
        usleep(2000);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Could not transmit shutdown brake: %s", exception.what());
    }
  }

  std::string chassis_type_;
  std::string can_interface_;
  std::string command_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double send_rate_hz_{10.0};
  double command_timeout_sec_{0.25};
  double feedback_timeout_sec_{1.2};
  bool require_feedback_before_motion_{true};
  bool require_autonomous_mode_{true};
  bool legacy_brake_byte_{true};
  bool invert_left_motor_{false};
  bool invert_right_motor_{false};
  bool headlight_{false};
  bool allow_unverified_ackermann_protocol_{false};
  uint32_t ackermann_command_id_{protocol::kReferenceAckermannCommandId};
  uint32_t ackermann_state_id_{protocol::kReferenceAckermannStateId};
  protocol::DifferentialKinematics differential_config_;
  protocol::AckermannKinematics ackermann_config_;

  int socket_fd_{-1};
  uint8_t transmit_counter_{0};
  std::unordered_map<uint32_t, uint8_t> receive_counters_;
  uint64_t checksum_errors_{0};
  uint64_t counter_errors_{0};
  uint64_t replay_frames_{0};
  uint64_t invalid_frames_{0};
  uint64_t transmit_errors_{0};
  uint64_t receive_errors_{0};

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time chassis_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time left_motor_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_motor_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ackermann_state_time_{0, 0, RCL_ROS_TIME};
  bool command_received_{false};
  bool chassis_state_received_{false};
  bool left_motor_received_{false};
  bool right_motor_received_{false};
  bool ackermann_state_received_{false};
  bool motion_command_active_{false};
  bool stop_frames_sent_{false};
  bool left_motor_update_{false};
  bool right_motor_update_{false};
  std::optional<protocol::ChassisState> chassis_state_;
  std::optional<protocol::MotorState> left_motor_state_;
  std::optional<protocol::MotorState> right_motor_state_;
  std::optional<protocol::AckermannState> ackermann_state_;
  double measured_linear_velocity_{0.0};
  double measured_angular_velocity_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_callback_;
  bool pre_shutdown_callback_registered_{false};
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<agribot_hardware_bringup::ChassisCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_can_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
