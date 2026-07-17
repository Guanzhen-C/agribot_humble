// Copyright 2017-2020 EAI Team
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "src/CYdLidar.h"
#include "std_srvs/srv/empty.hpp"

namespace
{
constexpr char kRos2DriverVersion[] = "1.0.2";
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(
    node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s",
    kRos2DriverVersion);

  CYdLidar laser;

  auto string_option = node->declare_parameter<std::string>("port", "/dev/ydlidar");
  laser.setlidaropt(LidarPropSerialPort, string_option.c_str(), string_option.size());

  string_option = node->declare_parameter<std::string>("ignore_array", "");
  laser.setlidaropt(LidarPropIgnoreArray, string_option.c_str(), string_option.size());

  const auto frame_id = node->declare_parameter<std::string>("frame_id", "laser_frame");

  auto integer_option = node->declare_parameter<int>("baudrate", 230400);
  laser.setlidaropt(LidarPropSerialBaudrate, &integer_option, sizeof(integer_option));

  integer_option = node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  laser.setlidaropt(LidarPropLidarType, &integer_option, sizeof(integer_option));

  integer_option = node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(LidarPropDeviceType, &integer_option, sizeof(integer_option));

  integer_option = node->declare_parameter<int>("sample_rate", 9);
  laser.setlidaropt(LidarPropSampleRate, &integer_option, sizeof(integer_option));

  integer_option = node->declare_parameter<int>("abnormal_check_count", 4);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &integer_option, sizeof(integer_option));

  integer_option = node->declare_parameter<int>("intensity_bit", 0);
  laser.setlidaropt(LidarPropIntenstiyBit, &integer_option, sizeof(integer_option));

  auto bool_option = node->declare_parameter<bool>("fixed_resolution", false);
  laser.setlidaropt(LidarPropFixedResolution, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("reversion", true);
  laser.setlidaropt(LidarPropReversion, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("inverted", true);
  laser.setlidaropt(LidarPropInverted, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("auto_reconnect", true);
  laser.setlidaropt(LidarPropAutoReconnect, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("isSingleChannel", false);
  laser.setlidaropt(LidarPropSingleChannel, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("intensity", false);
  laser.setlidaropt(LidarPropIntenstiy, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("support_motor_dtr", false);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_option, sizeof(bool_option));

  bool_option = node->declare_parameter<bool>("debug", false);
  laser.setEnableDebug(bool_option);

  auto float_option = node->declare_parameter<float>("angle_max", 180.0F);
  laser.setlidaropt(LidarPropMaxAngle, &float_option, sizeof(float_option));

  float_option = node->declare_parameter<float>("angle_min", -180.0F);
  laser.setlidaropt(LidarPropMinAngle, &float_option, sizeof(float_option));

  float_option = node->declare_parameter<float>("range_max", 64.0F);
  laser.setlidaropt(LidarPropMaxRange, &float_option, sizeof(float_option));

  float_option = node->declare_parameter<float>("range_min", 0.1F);
  laser.setlidaropt(LidarPropMinRange, &float_option, sizeof(float_option));

  float_option = node->declare_parameter<float>("frequency", 10.0F);
  laser.setlidaropt(LidarPropScanFrequency, &float_option, sizeof(float_option));

  // Retained for compatibility with existing parameter files.
  node->declare_parameter<bool>("invalid_range_is_inf", false);

  bool running = laser.initialize();
  if (running) {
    auto work_mode = node->declare_parameter<int>("m1_mode", 0);
    laser.setWorkMode(work_mode, 0x01);

    work_mode = node->declare_parameter<int>("m2_mode", 0);
    laser.setWorkMode(work_mode, 0x02);

    work_mode = node->declare_parameter<int>("m3_mode", 1);
    laser.setWorkMode(work_mode, 0x04);

    running = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }

  const auto laser_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

  const auto stop_service = node->create_service<std_srvs::srv::Empty>(
    "stop_scan",
    [&laser](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {laser.turnOff();});

  const auto start_service = node->create_service<std_srvs::srv::Empty>(
    "start_scan",
    [&laser](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {laser.turnOn();});

  rclcpp::WallRate loop_rate(20);
  while (running && rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      auto scan_message = std::make_unique<sensor_msgs::msg::LaserScan>();
      scan_message->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_message->header.stamp.nanosec =
        scan.stamp - RCL_S_TO_NS(scan_message->header.stamp.sec);
      scan_message->header.frame_id = frame_id;
      scan_message->angle_min = scan.config.min_angle;
      scan_message->angle_max = scan.config.max_angle;
      scan_message->angle_increment = scan.config.angle_increment;
      scan_message->scan_time = scan.config.scan_time;
      scan_message->time_increment = scan.config.time_increment;
      scan_message->range_min = scan.config.min_range;
      scan_message->range_max = scan.config.max_range;

      const int scan_size = static_cast<int>(
        (scan.config.max_angle - scan.config.min_angle) /
        scan.config.angle_increment) + 1;
      scan_message->ranges.resize(scan_size);
      scan_message->intensities.resize(scan_size);

      for (const auto & point : scan.points) {
        const int index = static_cast<int>(std::ceil(
            (point.angle - scan.config.min_angle) / scan.config.angle_increment));
        if (index >= 0 && index < scan_size) {
          scan_message->ranges[index] = point.range;
          scan_message->intensities[index] = point.intensity;
        }
      }

      laser_publisher->publish(std::move(scan_message));
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
