// Copyright 2026 cgz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace
{

constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kSync2 = 0xA5;
constexpr uint8_t kHi91Tag = 0x91;
constexpr std::size_t kHeaderSize = 6;
constexpr std::size_t kHi91PayloadSize = 76;
constexpr std::size_t kMaxPayloadSize = 512;
constexpr double kPi = 3.14159265358979323846;

uint16_t readU16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t readU32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

float readFloat(const uint8_t * data)
{
  const uint32_t bits = readU32(data);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t crc16Update(uint16_t crc, const uint8_t * data, std::size_t length)
{
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

speed_t baudToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
  }
}

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Quaternion multiply(const Quaternion & left, const Quaternion & right)
{
  return {
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
  };
}

Quaternion normalized(Quaternion quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return {};
  }
  quaternion.w /= norm;
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  return quaternion;
}

std::array<double, 3> rotateAboutZ(
  const std::array<float, 3> & input, double yaw_rad)
{
  const double cosine = std::cos(yaw_rad);
  const double sine = std::sin(yaw_rad);
  return {
    cosine * static_cast<double>(input[0]) - sine * static_cast<double>(input[1]),
    sine * static_cast<double>(input[0]) + cosine * static_cast<double>(input[1]),
    static_cast<double>(input[2]),
  };
}

Quaternion yawQuaternion(double yaw_rad)
{
  return {std::cos(yaw_rad / 2.0), 0.0, 0.0, std::sin(yaw_rad / 2.0)};
}

void setDiagonal(std::array<double, 9> & covariance, double standard_deviation)
{
  covariance.fill(0.0);
  const double variance = standard_deviation * standard_deviation;
  covariance[0] = variance;
  covariance[4] = variance;
  covariance[8] = variance;
}

struct Hi91Sample
{
  int8_t temperature = 0;
  uint32_t timestamp_ms = 0;
  std::array<float, 3> acceleration_g{};
  std::array<float, 3> angular_velocity_deg_s{};
  std::array<float, 3> magnetic_field_ut{};
  Quaternion enu_from_rfu;
};

}  // namespace

class HipnucImuNode : public rclcpp::Node
{
public:
  HipnucImuNode()
  : Node("hipnuc_imu")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    magnetic_topic_ =
      declare_parameter<std::string>("magnetic_topic", "/imu/magnetic_field");
    temperature_topic_ =
      declare_parameter<std::string>("temperature_topic", "/imu/temperature");
    publish_magnetic_field_ = declare_parameter<bool>("publish_magnetic_field", true);
    publish_temperature_ = declare_parameter<bool>("publish_temperature", true);
    gravity_m_s2_ = declare_parameter<double>("gravity_m_s2", 9.80665);
    const double device_yaw_in_flu_deg =
      declare_parameter<double>("device_yaw_in_flu_deg", -90.0);
    if (!std::isfinite(device_yaw_in_flu_deg)) {
      throw std::invalid_argument("device_yaw_in_flu_deg must be finite");
    }
    device_yaw_in_flu_rad_ = device_yaw_in_flu_deg * kPi / 180.0;
    device_from_flu_ = yawQuaternion(-device_yaw_in_flu_rad_);

    const double orientation_std_rad =
      declare_parameter<double>("orientation_std_deg", 2.0) * kPi / 180.0;
    const double angular_velocity_std_rad_s =
      declare_parameter<double>("angular_velocity_std_deg_s", 0.1) * kPi / 180.0;
    const double linear_acceleration_std_m_s2 =
      declare_parameter<double>("linear_acceleration_std_m_s2", 0.08);
    const double magnetic_field_std_t =
      declare_parameter<double>("magnetic_field_std_ut", 1.0) * 1e-6;
    setDiagonal(orientation_covariance_, orientation_std_rad);
    setDiagonal(angular_velocity_covariance_, angular_velocity_std_rad_s);
    setDiagonal(linear_acceleration_covariance_, linear_acceleration_std_m_s2);
    setDiagonal(magnetic_field_covariance_, magnetic_field_std_t);
    RCLCPP_INFO(
      get_logger(), "N300Pro frame conversion: raw +X yaw in FLU = %.1f deg",
      device_yaw_in_flu_deg);

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 50);
    if (publish_magnetic_field_) {
      magnetic_publisher_ =
        create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic_, 20);
    }
    if (publish_temperature_) {
      temperature_publisher_ =
        create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_, 10);
    }

    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&HipnucImuNode::pollSerial, this));
    openSerial();
  }

  ~HipnucImuNode() override {closeSerial();}

private:
  void openSerial()
  {
    last_open_attempt_ = now();
    closeSerial();
    try {
      serial_fd_ =
        open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (serial_fd_ < 0) {
        throw std::runtime_error(std::strerror(errno));
      }

      termios options{};
      if (tcgetattr(serial_fd_, &options) != 0) {
        throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
      }
      cfmakeraw(&options);
      const speed_t speed = baudToTermios(baud_rate_);
      cfsetispeed(&options, speed);
      cfsetospeed(&options, speed);
      options.c_cflag |= CLOCAL | CREAD;
      options.c_cflag &= ~CRTSCTS;
      options.c_cc[VMIN] = 0;
      options.c_cc[VTIME] = 0;
      if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
      }
      tcflush(serial_fd_, TCIFLUSH);
      receive_buffer_.clear();
      RCLCPP_INFO(
        get_logger(), "N300Pro connected: port=%s baud=%d topic=%s frame=%s",
        serial_port_.c_str(), baud_rate_, imu_topic_.c_str(), frame_id_.c_str());
    } catch (const std::exception & exception) {
      closeSerial();
      RCLCPP_ERROR(
        get_logger(), "Cannot open N300Pro serial port %s: %s; retrying",
        serial_port_.c_str(), exception.what());
    }
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void pollSerial()
  {
    if (serial_fd_ < 0) {
      if ((now() - last_open_attempt_).seconds() >= 1.0) {
        openSerial();
      }
      return;
    }

    std::array<uint8_t, 4096> buffer{};
    while (true) {
      const ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
      if (bytes_read > 0) {
        receive_buffer_.insert(
          receive_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
        parseFrames();
        continue;
      }
      if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      RCLCPP_ERROR(get_logger(), "N300Pro serial read failed: %s", std::strerror(errno));
      closeSerial();
      break;
    }
  }

  void parseFrames()
  {
    static constexpr std::array<uint8_t, 2> kSync{kSync1, kSync2};
    while (receive_buffer_.size() >= kHeaderSize) {
      const auto sync = std::search(
        receive_buffer_.begin(), receive_buffer_.end(),
        kSync.begin(), kSync.end());
      if (sync != receive_buffer_.begin()) {
        receive_buffer_.erase(receive_buffer_.begin(), sync);
        if (receive_buffer_.size() < kHeaderSize) {
          return;
        }
      }

      const std::size_t payload_size = readU16(receive_buffer_.data() + 2);
      if (payload_size == 0 || payload_size > kMaxPayloadSize) {
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }
      const std::size_t frame_size = kHeaderSize + payload_size;
      if (receive_buffer_.size() < frame_size) {
        return;
      }

      uint16_t calculated_crc = crc16Update(0, receive_buffer_.data(), 4);
      calculated_crc = crc16Update(
        calculated_crc, receive_buffer_.data() + kHeaderSize, payload_size);
      const uint16_t expected_crc = readU16(receive_buffer_.data() + 4);
      if (calculated_crc == expected_crc) {
        decodePayload(receive_buffer_.data() + kHeaderSize, payload_size);
      } else {
        ++crc_error_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "N300Pro CRC errors detected (total=%zu)", crc_error_count_);
      }
      receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frame_size);
    }
  }

  void decodePayload(const uint8_t * payload, std::size_t payload_size)
  {
    if (payload_size < kHi91PayloadSize || payload[0] != kHi91Tag) {
      return;
    }

    Hi91Sample sample;
    sample.temperature = static_cast<int8_t>(payload[3]);
    sample.timestamp_ms = readU32(payload + 8);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sample.acceleration_g[axis] = readFloat(payload + 12 + axis * 4);
      sample.angular_velocity_deg_s[axis] = readFloat(payload + 24 + axis * 4);
      sample.magnetic_field_ut[axis] = readFloat(payload + 36 + axis * 4);
    }
    sample.enu_from_rfu = normalized(
      {readFloat(payload + 60), readFloat(payload + 64),
        readFloat(payload + 68), readFloat(payload + 72)});
    publish(sample);
  }

  void publish(const Hi91Sample & sample)
  {
    const rclcpp::Time stamp = now();
    const auto acceleration_flu =
      rotateAboutZ(sample.acceleration_g, device_yaw_in_flu_rad_);
    const auto angular_velocity_flu =
      rotateAboutZ(sample.angular_velocity_deg_s, device_yaw_in_flu_rad_);
    const auto magnetic_field_flu =
      rotateAboutZ(sample.magnetic_field_ut, device_yaw_in_flu_rad_);

    // The device reports ENU<-device. Apply the same mounting rotation used
    // for vectors so the orientation remains ENU<-FLU.
    const Quaternion enu_from_flu =
      normalized(multiply(sample.enu_from_rfu, device_from_flu_));

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;
    imu.orientation.w = enu_from_flu.w;
    imu.orientation.x = enu_from_flu.x;
    imu.orientation.y = enu_from_flu.y;
    imu.orientation.z = enu_from_flu.z;
    imu.angular_velocity.x = angular_velocity_flu[0] * kPi / 180.0;
    imu.angular_velocity.y = angular_velocity_flu[1] * kPi / 180.0;
    imu.angular_velocity.z = angular_velocity_flu[2] * kPi / 180.0;
    imu.linear_acceleration.x = acceleration_flu[0] * gravity_m_s2_;
    imu.linear_acceleration.y = acceleration_flu[1] * gravity_m_s2_;
    imu.linear_acceleration.z = acceleration_flu[2] * gravity_m_s2_;
    imu.orientation_covariance = orientation_covariance_;
    imu.angular_velocity_covariance = angular_velocity_covariance_;
    imu.linear_acceleration_covariance = linear_acceleration_covariance_;
    imu_publisher_->publish(imu);

    if (magnetic_publisher_) {
      sensor_msgs::msg::MagneticField magnetic;
      magnetic.header = imu.header;
      magnetic.magnetic_field.x = magnetic_field_flu[0] * 1e-6;
      magnetic.magnetic_field.y = magnetic_field_flu[1] * 1e-6;
      magnetic.magnetic_field.z = magnetic_field_flu[2] * 1e-6;
      magnetic.magnetic_field_covariance = magnetic_field_covariance_;
      magnetic_publisher_->publish(magnetic);
    }

    if (temperature_publisher_) {
      sensor_msgs::msg::Temperature temperature;
      temperature.header = imu.header;
      temperature.temperature = static_cast<double>(sample.temperature);
      temperature.variance = 1.0;
      temperature_publisher_->publish(temperature);
    }
  }

  std::string serial_port_;
  int baud_rate_ = 115200;
  std::string frame_id_;
  std::string imu_topic_;
  std::string magnetic_topic_;
  std::string temperature_topic_;
  bool publish_magnetic_field_ = true;
  bool publish_temperature_ = true;
  double gravity_m_s2_ = 9.80665;
  double device_yaw_in_flu_rad_ = -kPi / 2.0;
  Quaternion device_from_flu_{std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};

  int serial_fd_ = -1;
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};
  std::vector<uint8_t> receive_buffer_;
  std::size_t crc_error_count_ = 0;
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
  std::array<double, 9> magnetic_field_covariance_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HipnucImuNode>());
  rclcpp::shutdown();
  return 0;
}
