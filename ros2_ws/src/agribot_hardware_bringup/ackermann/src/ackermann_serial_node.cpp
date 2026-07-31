#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scout_msgs/msg/scout_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"
#include "agribot_hardware_bringup/ackermann_serial_protocol.hpp"

namespace agribot_hardware_bringup
{
namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

speed_t baudConstant(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      throw std::invalid_argument("unsupported serial baud rate");
  }
}

diagnostic_msgs::msg::KeyValue keyValue(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

class AckermannSerialNode final : public rclcpp::Node
{
public:
  AckermannSerialNode()
  : Node("ackermann_chassis_serial")
  {
    port_ = declare_parameter<std::string>(
      "port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C2C079857-if00");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    command_topic_ = declare_parameter<std::string>("command_topic", "/hardware/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "wheel_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    chassis_imu_topic_ =
      declare_parameter<std::string>("chassis_imu_topic", "/hardware/chassis_imu");
    chassis_imu_frame_ = declare_parameter<std::string>("chassis_imu_frame", "base_link");
    battery_topic_ =
      declare_parameter<std::string>("battery_topic", "/hardware/battery_voltage");
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 20.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.6);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 1.0);
    require_feedback_before_motion_ =
      declare_parameter<bool>("require_feedback_before_motion", true);
    require_localization_ready_ =
      declare_parameter<bool>("require_localization_ready", false);
    localization_ready_topic_ =
      declare_parameter<std::string>(
      "localization_ready_topic", "/localization/ready");
    localization_ready_timeout_sec_ =
      declare_parameter<double>("localization_ready_timeout_sec", 2.5);

    kinematics_.wheelbase_m = declare_parameter<double>("wheelbase_m", 0.5265855);
    kinematics_.max_steering_angle_rad =
      declare_parameter<double>("max_steering_angle_rad", 0.384);
    kinematics_.max_linear_velocity =
      declare_parameter<double>("max_linear_velocity", 0.80);
    kinematics_.max_angular_velocity =
      declare_parameter<double>("max_angular_velocity", 0.613854);
    kinematics_.minimum_motion_speed =
      declare_parameter<double>("minimum_motion_speed", 0.02);
    max_steering_rate_rad_s_ =
      declare_parameter<double>("max_steering_rate_rad_s", 0.60);
    validateParameters();

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 20,
      std::bind(&AckermannSerialNode::handleCommand, this, std::placeholders::_1));
    if (require_localization_ready_) {
      localization_ready_subscription_ =
        create_subscription<std_msgs::msg::Bool>(
        localization_ready_topic_,
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(
          &AckermannSerialNode::handleLocalizationReady,
          this, std::placeholders::_1));
    }
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    status_publisher_ = create_publisher<scout_msgs::msg::ScoutStatus>("/scout_status", 10);
    chassis_imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(chassis_imu_topic_, 20);
    battery_publisher_ = create_publisher<std_msgs::msg::Float32>(battery_topic_, 10);
    emergency_stop_publisher_ =
      create_publisher<std_msgs::msg::Bool>("/hardware/chassis_e_stop", 10);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());

    const auto send_period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(send_period),
      std::bind(&AckermannSerialNode::sendCommand, this));
    receive_timer_ =
      create_wall_timer(5ms, std::bind(&AckermannSerialNode::receiveTelemetry, this));
    diagnostics_timer_ =
      create_wall_timer(1s, std::bind(&AckermannSerialNode::publishDiagnostics, this));

    context_ = get_node_base_interface()->get_context();
    pre_shutdown_callback_ = context_->add_pre_shutdown_callback(
      [this]() {
        RCLCPP_WARN(get_logger(), "ROS shutdown requested; transmitting serial stop frames");
        sendStopFrames();
      });
    pre_shutdown_callback_registered_ = true;

    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      openSerial();
    }
    RCLCPP_INFO(
      get_logger(),
      "Ackermann serial chassis ready: port=%s baud=%d command=%s rate=%.1f Hz "
      "steering_rate=%.2f rad/s",
      port_.c_str(), baud_rate_, command_topic_.c_str(), send_rate_hz_,
      max_steering_rate_rad_s_);
  }

  ~AckermannSerialNode() override
  {
    if (pre_shutdown_callback_registered_ && context_) {
      context_->remove_pre_shutdown_callback(pre_shutdown_callback_);
    }
    sendStopFrames();
    std::lock_guard<std::mutex> guard(state_mutex_);
    closeSerial();
  }

private:
  void validateParameters()
  {
    if (port_.empty()) {
      throw std::invalid_argument("serial port must not be empty");
    }
    (void)baudConstant(baud_rate_);
    if (send_rate_hz_ <= 0.0 || command_timeout_sec_ <= 0.0 ||
      feedback_timeout_sec_ <= 0.0 || reconnect_interval_sec_ <= 0.0 ||
      localization_ready_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("serial timing parameters must be positive");
    }
    if (!std::isfinite(max_steering_rate_rad_s_) ||
      max_steering_rate_rad_s_ <= 0.0)
    {
      throw std::invalid_argument("max_steering_rate_rad_s must be positive");
    }
    (void)ackermann_can::fromTwist(0.0, 0.0, kinematics_, true);
  }

  void configureSerial(int descriptor)
  {
    struct termios options {};
    if (tcgetattr(descriptor, &options) != 0) {
      throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }
    cfmakeraw(&options);
    const auto speed = baudConstant(baud_rate_);
    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
      throw std::system_error(errno, std::generic_category(), "cfsetspeed");
    }
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
    options.c_cflag |= CS8;
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor, TCSANOW, &options) != 0) {
      throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }
    if (tcflush(descriptor, TCIOFLUSH) != 0) {
      throw std::system_error(errno, std::generic_category(), "tcflush");
    }
  }

  void openSerial()
  {
    last_open_attempt_ = SteadyClock::now();
    const int descriptor =
      open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot open %s: %s",
        port_.c_str(), std::strerror(errno));
      return;
    }

    try {
      if (ioctl(descriptor, TIOCEXCL) != 0) {
        throw std::system_error(errno, std::generic_category(), "TIOCEXCL");
      }
      configureSerial(descriptor);
    } catch (const std::exception & exception) {
      close(descriptor);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot configure %s: %s",
        port_.c_str(), exception.what());
      return;
    }

    serial_fd_ = descriptor;
    parser_.reset();
    feedback_received_ = false;
    last_odom_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_steering_update_time_ = SteadyClock::time_point{};
    sent_steering_angle_rad_ = 0.0;
    ++open_count_;
    publishEmergencyStop(true);
    RCLCPP_INFO(get_logger(), "Opened serial chassis %s at %d", port_.c_str(), baud_rate_);
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    feedback_received_ = false;
    motion_command_active_ = false;
    last_steering_update_time_ = SteadyClock::time_point{};
    sent_steering_angle_rad_ = 0.0;
  }

  bool shouldReconnect(SteadyClock::time_point current_time) const
  {
    return serial_fd_ < 0 &&
           std::chrono::duration<double>(current_time - last_open_attempt_).count() >=
           reconnect_interval_sec_;
  }

  void handleCommand(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      std::lock_guard<std::mutex> guard(state_mutex_);
      command_received_ = false;
      RCLCPP_ERROR(get_logger(), "Rejected non-finite chassis command");
      return;
    }
    std::lock_guard<std::mutex> guard(state_mutex_);
    latest_command_ = *message;
    last_command_time_ = SteadyClock::now();
    command_received_ = true;
  }

  void handleLocalizationReady(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    localization_ready_received_ = true;
    localization_ready_ = message->data;
    last_localization_ready_time_ = SteadyClock::now();
  }

  bool commandFresh(SteadyClock::time_point current_time) const
  {
    return command_received_ &&
           std::chrono::duration<double>(current_time - last_command_time_).count() <=
           command_timeout_sec_;
  }

  bool feedbackFresh(SteadyClock::time_point current_time) const
  {
    return feedback_received_ &&
           std::chrono::duration<double>(current_time - last_feedback_time_).count() <=
           feedback_timeout_sec_;
  }

  bool localizationAllowsMotion(SteadyClock::time_point current_time) const
  {
    if (!require_localization_ready_) {
      return true;
    }
    return localization_ready_received_ && localization_ready_ &&
           std::chrono::duration<double>(
      current_time - last_localization_ready_time_).count() <=
           localization_ready_timeout_sec_;
  }

  void writeFrame(const ackermann_serial::CommandFrame & frame)
  {
    const auto bytes = write(serial_fd_, frame.data(), frame.size());
    if (bytes != static_cast<ssize_t>(frame.size())) {
      const int error = bytes < 0 ? errno : EIO;
      throw std::system_error(error, std::generic_category(), "write(serial)");
    }
    ++transmit_frames_;
  }

  void sendCommand()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto current_time = SteadyClock::now();
    if (shouldReconnect(current_time)) {
      openSerial();
    }
    if (serial_fd_ < 0) {
      return;
    }

    const bool feedback_ready =
      !require_feedback_before_motion_ || feedbackFresh(current_time);
    const bool motion_allowed =
      commandFresh(current_time) && feedback_ready &&
      localizationAllowsMotion(current_time);
    try {
      auto command = ackermann_can::fromTwist(
        latest_command_.linear.x, latest_command_.angular.z,
        kinematics_, !motion_allowed);
      requested_steering_angle_rad_ = command.steering_angle_rad;
      steering_rate_limited_ = false;
      if (motion_allowed &&
        std::abs(command.speed_mps) >= kinematics_.minimum_motion_speed)
      {
        double elapsed_sec = 1.0 / send_rate_hz_;
        if (last_steering_update_time_ != SteadyClock::time_point{}) {
          elapsed_sec =
            std::chrono::duration<double>(
            current_time - last_steering_update_time_).count();
        }
        command.steering_angle_rad = ackermann_can::limitSteeringRate(
          command.steering_angle_rad,
          sent_steering_angle_rad_,
          max_steering_rate_rad_s_,
          elapsed_sec);
        steering_rate_limited_ =
          std::abs(command.steering_angle_rad - requested_steering_angle_rad_) > 1e-9;
        if (steering_rate_limited_) {
          ++steering_rate_limit_count_;
        }
      }
      writeFrame(ackermann_serial::encodeCommand(command));
      sent_steering_angle_rad_ = command.steering_angle_rad;
      last_steering_update_time_ = current_time;
      motion_command_active_ =
        motion_allowed &&
        (std::abs(command.speed_mps) >= kinematics_.minimum_motion_speed);
    } catch (const std::exception & exception) {
      ++write_errors_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Serial chassis transmit failed: %s",
        exception.what());
      closeSerial();
      publishEmergencyStop(true);
    }
  }

  void receiveTelemetry()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto steady_now = SteadyClock::now();
    if (shouldReconnect(steady_now)) {
      openSerial();
    }
    if (serial_fd_ < 0) {
      return;
    }

    std::array<uint8_t, 4096> buffer{};
    for (std::size_t iteration = 0; iteration < 8U; ++iteration) {
      const auto bytes = read(serial_fd_, buffer.data(), buffer.size());
      if (bytes > 0) {
        const auto decoded =
          parser_.feed(buffer.data(), static_cast<std::size_t>(bytes));
        for (const auto & telemetry : decoded) {
          processTelemetry(telemetry);
        }
        continue;
      }
      if (bytes == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      ++read_errors_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Serial chassis receive failed: %s",
        std::strerror(errno));
      closeSerial();
      publishEmergencyStop(true);
      return;
    }
  }

  void processTelemetry(const ackermann_can::Telemetry & telemetry)
  {
    const auto stamp = now();
    last_feedback_time_ = SteadyClock::now();
    feedback_received_ = true;
    ++receive_frames_;

    updateOdometry(
      stamp, telemetry.linear_velocity_x, telemetry.angular_velocity_z);

    scout_msgs::msg::ScoutStatus status;
    status.stamp = stamp;
    status.linear_velocity = telemetry.linear_velocity_x;
    status.angular_velocity = telemetry.angular_velocity_z;
    status.battery_voltage = telemetry.battery_voltage;
    status_publisher_->publish(status);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = chassis_imu_frame_;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = telemetry.angular_velocity_x;
    imu.angular_velocity.y = telemetry.angular_velocity_y;
    imu.angular_velocity.z = telemetry.imu_angular_velocity_z;
    imu.linear_acceleration.x = telemetry.linear_acceleration_x;
    imu.linear_acceleration.y = telemetry.linear_acceleration_y;
    imu.linear_acceleration.z = telemetry.linear_acceleration_z;
    chassis_imu_publisher_->publish(imu);

    std_msgs::msg::Float32 battery;
    battery.data = static_cast<float>(telemetry.battery_voltage);
    battery_publisher_->publish(battery);
    publishEmergencyStop(false);
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

  void publishEmergencyStop(bool stopped)
  {
    std_msgs::msg::Bool state;
    state.data = stopped;
    emergency_stop_publisher_->publish(state);
  }

  void publishDiagnostics()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto current_time = SteadyClock::now();
    const bool fresh = serial_fd_ >= 0 && feedbackFresh(current_time);
    publishEmergencyStop(!fresh);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agribot/chassis_serial/ackermann";
    status.hardware_id = port_;
    if (serial_fd_ < 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial chassis port unavailable";
    } else if (!fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial chassis feedback missing or stale";
    } else if (!localizationAllowsMotion(current_time)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for healthy map localization";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "serial protocol and feedback healthy";
    }
    status.values.push_back(keyValue("port_open", serial_fd_ >= 0 ? "true" : "false"));
    status.values.push_back(keyValue("feedback_fresh", fresh ? "true" : "false"));
    status.values.push_back(
      keyValue("command_active", motion_command_active_ ? "true" : "false"));
    status.values.push_back(
      keyValue(
        "localization_ready",
        localizationAllowsMotion(current_time) ? "true" : "false"));
    status.values.push_back(keyValue("received_frames", std::to_string(receive_frames_)));
    status.values.push_back(
      keyValue("transmitted_frames", std::to_string(transmit_frames_)));
    status.values.push_back(
      keyValue("invalid_frames", std::to_string(parser_.invalidFrames())));
    status.values.push_back(
      keyValue("discarded_bytes", std::to_string(parser_.discardedBytes())));
    status.values.push_back(keyValue("open_count", std::to_string(open_count_)));
    status.values.push_back(keyValue("read_errors", std::to_string(read_errors_)));
    status.values.push_back(keyValue("write_errors", std::to_string(write_errors_)));
    status.values.push_back(
      keyValue(
        "max_steering_rate_rad_s",
        std::to_string(max_steering_rate_rad_s_)));
    status.values.push_back(
      keyValue(
        "requested_steering_angle_rad",
        std::to_string(requested_steering_angle_rad_)));
    status.values.push_back(
      keyValue(
        "sent_steering_angle_rad",
        std::to_string(sent_steering_angle_rad_)));
    status.values.push_back(
      keyValue(
        "steering_rate_limited",
        steering_rate_limited_ ? "true" : "false"));
    status.values.push_back(
      keyValue(
        "steering_rate_limit_count",
        std::to_string(steering_rate_limit_count_)));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  void sendStopFrames() noexcept
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    if (serial_fd_ < 0 || stop_frames_sent_) {
      return;
    }
    stop_frames_sent_ = true;
    try {
      const auto stopped = ackermann_can::fromTwist(0.0, 0.0, kinematics_, true);
      const auto frame = ackermann_serial::encodeCommand(stopped);
      for (int count = 0; count < 10; ++count) {
        writeFrame(frame);
        usleep(20000);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Could not transmit serial shutdown stop: %s", exception.what());
    }
  }

  std::string port_;
  int baud_rate_{115200};
  std::string command_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string chassis_imu_topic_;
  std::string chassis_imu_frame_;
  std::string battery_topic_;
  double send_rate_hz_{20.0};
  double command_timeout_sec_{0.25};
  double feedback_timeout_sec_{0.6};
  double reconnect_interval_sec_{1.0};
  double localization_ready_timeout_sec_{2.5};
  double max_steering_rate_rad_s_{0.60};
  bool require_feedback_before_motion_{true};
  bool require_localization_ready_{false};
  std::string localization_ready_topic_{"/localization/ready"};
  ackermann_can::Kinematics kinematics_;

  int serial_fd_{-1};
  ackermann_serial::TelemetryParser parser_;
  uint64_t receive_frames_{0U};
  uint64_t transmit_frames_{0U};
  uint64_t open_count_{0U};
  uint64_t read_errors_{0U};
  uint64_t write_errors_{0U};
  uint64_t steering_rate_limit_count_{0U};

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist latest_command_;
  SteadyClock::time_point last_command_time_{};
  SteadyClock::time_point last_feedback_time_{};
  SteadyClock::time_point last_open_attempt_{};
  SteadyClock::time_point last_localization_ready_time_{};
  SteadyClock::time_point last_steering_update_time_{};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  bool command_received_{false};
  bool feedback_received_{false};
  bool localization_ready_received_{false};
  bool localization_ready_{false};
  bool motion_command_active_{false};
  bool steering_rate_limited_{false};
  bool stop_frames_sent_{false};
  double requested_steering_angle_rad_{0.0};
  double sent_steering_angle_rad_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    localization_ready_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr chassis_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
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
}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<agribot_hardware_bringup::AckermannSerialNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ackermann_chassis_serial"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
