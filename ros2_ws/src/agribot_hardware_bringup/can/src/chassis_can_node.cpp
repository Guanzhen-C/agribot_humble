#include "agribot_hardware_bringup/chassis_can_node.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scout_msgs/msg/scout_status.hpp"
#include "std_msgs/msg/bool.hpp"

namespace agribot_hardware_bringup
{
namespace
{

using namespace std::chrono_literals;

class ChassisCanNode final : public rclcpp::Node
{
public:
  ChassisCanNode(const char * node_name, ChassisAdapterFactory adapter_factory)
  : Node(node_name)
  {
    if (adapter_factory == nullptr) {
      throw std::invalid_argument("chassis adapter factory is required");
    }

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
    headlight_ = declare_parameter<bool>("headlight", false);

    adapter_ = adapter_factory(*this);
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
      adapter_->type().c_str(), can_interface_.c_str(), command_topic_.c_str(), send_rate_hz_);
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
    if (send_rate_hz_ <= 0.0 || command_timeout_sec_ <= 0.0 ||
      feedback_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("CAN timing parameters must be positive");
    }

    std::vector<uint32_t> ids{adapter_->commandId()};
    const auto feedback_ids = adapter_->feedbackIds();
    if (feedback_ids.empty()) {
      throw std::invalid_argument("at least one CAN feedback ID is required");
    }
    ids.insert(ids.end(), feedback_ids.begin(), feedback_ids.end());
    if (std::any_of(ids.begin(), ids.end(), [](uint32_t id) {return id > CAN_SFF_MASK;})) {
      throw std::invalid_argument("only standard 11-bit CAN IDs are supported");
    }
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
      throw std::invalid_argument("chassis CAN IDs must be unique");
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

    const auto feedback_ids = adapter_->feedbackIds();
    std::vector<struct can_filter> filters;
    filters.reserve(feedback_ids.size());
    for (const auto id : feedback_ids) {
      filters.push_back({id, CAN_SFF_MASK});
    }
    if (setsockopt(
        socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0)
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
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      std::lock_guard<std::mutex> guard(state_mutex_);
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
    return adapter_->feedbackFresh(current_time, feedback_timeout_sec_);
  }

  bool feedbackAllowsMotion(const rclcpp::Time & current_time) const
  {
    if (!require_feedback_before_motion_) {
      return true;
    }
    return feedbackFresh(current_time) &&
           adapter_->feedbackAllowsMotion(require_autonomous_mode_);
  }

  void sendCommand()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto current_time = now();
    const double command_age = (current_time - last_command_time_).seconds();
    const bool command_fresh = command_received_ && command_age >= 0.0 &&
      command_age <= command_timeout_sec_;
    const bool motion_allowed = command_fresh && feedbackAllowsMotion(current_time);

    try {
      const auto frame = adapter_->commandFromTwist(
        latest_command_, !motion_allowed, headlight_, transmit_counter_);
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

  void writeFrame(const chassis_can::Frame & frame)
  {
    struct can_frame raw_frame {};
    raw_frame.can_id = frame.id;
    raw_frame.can_dlc = chassis_can::kPayloadSize;
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
        raw_frame.can_dlc != chassis_can::kPayloadSize ||
        (raw_frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U)
      {
        ++invalid_frames_;
        continue;
      }

      chassis_can::Frame frame;
      frame.id = raw_frame.can_id & CAN_SFF_MASK;
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), frame.data.begin());
      processFrame(frame);
    }
  }

  bool counterIsNew(const chassis_can::Frame & frame)
  {
    const auto counter = chassis_can::rollingCounter(frame.data);
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

  void processFrame(const chassis_can::Frame & frame)
  {
    if (adapter_->usesPerFrameIntegrity()) {
      if (!chassis_can::hasValidChecksum(frame.data)) {
        ++checksum_errors_;
        return;
      }
      if (!counterIsNew(frame)) {
        return;
      }
    }

    const auto current_time = now();
    const auto update = adapter_->processFrame(frame, current_time);
    if (!update.valid) {
      if (update.checksum_error) {
        ++checksum_errors_;
      } else {
        ++invalid_frames_;
      }
      return;
    }
    if (update.emergency_stop.has_value()) {
      std_msgs::msg::Bool emergency;
      emergency.data = *update.emergency_stop;
      emergency_stop_publisher_->publish(emergency);
    }
    if (update.motion.has_value()) {
      updateOdometry(
        current_time, update.motion->linear_velocity, update.motion->angular_velocity);
    }
    publishStatus();
  }

  void updateOdometry(
    const rclcpp::Time & stamp,
    double linear_velocity,
    double angular_velocity)
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
    adapter_->populateStatus(status);
    status_publisher_->publish(status);
  }

  static diagnostic_msgs::msg::KeyValue keyValue(
    const std::string & key,
    const std::string & value)
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
    status.name = "agribot/chassis_can/" + adapter_->type();
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
    status.values.push_back(keyValue("chassis_type", adapter_->type()));
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
      geometry_msgs::msg::Twist stopped;
      for (int count = 0; count < 3; ++count) {
        const auto frame = adapter_->commandFromTwist(
          stopped, true, headlight_, transmit_counter_);
        writeFrame(frame);
        transmit_counter_ = (transmit_counter_ + 1U) & 0x0fU;
        usleep(2000);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Could not transmit shutdown brake: %s", exception.what());
    }
  }

  std::unique_ptr<ChassisAdapter> adapter_;
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
  bool headlight_{false};

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
  bool command_received_{false};
  bool motion_command_active_{false};
  bool stop_frames_sent_{false};
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

}  // namespace

int runChassisCanNode(
  int argc,
  char ** argv,
  const char * node_name,
  ChassisAdapterFactory adapter_factory)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ChassisCanNode>(node_name, adapter_factory);
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger(node_name), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

}  // namespace agribot_hardware_bringup
