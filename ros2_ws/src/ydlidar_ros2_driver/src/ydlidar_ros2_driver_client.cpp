// Copyright 2017-2020 EAI Team
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

namespace
{
void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  const int count = static_cast<int>(scan->scan_time / scan->time_increment);
  std::printf(
    "[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n",
    scan->header.frame_id.c_str(), count);
  std::printf(
    "[YDLIDAR INFO]: angle_range : [%f, %f]\n",
    RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  for (int index = 0; index < count; ++index) {
    const float degree = RAD2DEG(scan->angle_min + scan->angle_increment * index);
    std::printf(
      "[YDLIDAR INFO]: angle-distance : [%f, %f]\n",
      degree, scan->ranges[index]);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");
  const auto lidar_info_subscription =
    node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), ScanCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
