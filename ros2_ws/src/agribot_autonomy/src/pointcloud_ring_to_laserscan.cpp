#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const auto & field) {return field.name == name;});
  return it == cloud.fields.end() ? nullptr : &*it;
}

template<typename T>
T read_value(const uint8_t * data)
{
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

bool read_ring(
  const uint8_t * point, const sensor_msgs::msg::PointField & field,
  int64_t & value)
{
  const uint8_t * data = point + field.offset;
  using sensor_msgs::msg::PointField;
  switch (field.datatype) {
    case PointField::INT8:
      value = read_value<int8_t>(data);
      return true;
    case PointField::UINT8:
      value = read_value<uint8_t>(data);
      return true;
    case PointField::INT16:
      value = read_value<int16_t>(data);
      return true;
    case PointField::UINT16:
      value = read_value<uint16_t>(data);
      return true;
    case PointField::INT32:
      value = read_value<int32_t>(data);
      return true;
    case PointField::UINT32:
      value = read_value<uint32_t>(data);
      return true;
    default:
      return false;
  }
}

}  // namespace

class PointcloudRingToLaserscan : public rclcpp::Node
{
public:
  PointcloudRingToLaserscan()
  : Node("pointcloud_ring_to_laserscan")
  {
    input_topic_ = declare_parameter<std::string>("input_cloud_topic", "/points");
    output_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan");
    ring_index_ = declare_parameter<int>("ring_index", 8);
    beam_count_ = declare_parameter<int>("beam_count", 720);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    range_min_ = declare_parameter<double>("range_min", 0.3);
    range_max_ = declare_parameter<double>("range_max", 25.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);

    if (beam_count_ < 1 || angle_max_ <= angle_min_ || range_max_ <= range_min_) {
      throw std::invalid_argument("invalid LaserScan geometry parameters");
    }

    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudRingToLaserscan::handle_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Converting ring %d from %s to %s (%d beams)",
      ring_index_, input_topic_.c_str(), output_topic_.c_str(), beam_count_);
  }

private:
  void handle_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    if (cloud->is_bigendian) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "big-endian PointCloud2 is not supported");
      return;
    }

    const auto * x_field = find_field(*cloud, "x");
    const auto * y_field = find_field(*cloud, "y");
    const auto * ring_field = find_field(*cloud, "ring");
    if (x_field == nullptr || y_field == nullptr || ring_field == nullptr) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 must contain x, y, and ring fields");
      return;
    }
    if (
      x_field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      y_field->datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 x and y fields must be FLOAT32");
      return;
    }

    const size_t point_count = static_cast<size_t>(cloud->width) * cloud->height;
    if (
      cloud->point_step == 0 ||
      cloud->data.size() < point_count * static_cast<size_t>(cloud->point_step))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 data length is inconsistent");
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud->header;
    const double span = angle_max_ - angle_min_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_increment = static_cast<float>(span / beam_count_);
    scan.angle_max = scan.angle_min + scan.angle_increment * (beam_count_ - 1);
    scan.scan_time = static_cast<float>(scan_time_);
    scan.time_increment = static_cast<float>(scan_time_ / beam_count_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(beam_count_, std::numeric_limits<float>::infinity());

    for (size_t index = 0; index < point_count; ++index) {
      const uint8_t * point = cloud->data.data() + index * cloud->point_step;
      int64_t ring = -1;
      if (!read_ring(point, *ring_field, ring)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "PointCloud2 ring field has unsupported type");
        return;
      }
      if (ring != ring_index_) {
        continue;
      }

      const float x = read_value<float>(point + x_field->offset);
      const float y = read_value<float>(point + y_field->offset);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      const float range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      double normalized_angle = std::fmod(std::atan2(y, x) - angle_min_, span);
      if (normalized_angle < 0.0) {
        normalized_angle += span;
      }
      const size_t beam = std::min(
        static_cast<size_t>(normalized_angle / span * beam_count_),
        static_cast<size_t>(beam_count_ - 1));
      scan.ranges[beam] = std::min(scan.ranges[beam], range);
    }

    scan_publisher_->publish(scan);
  }

  std::string input_topic_;
  std::string output_topic_;
  int ring_index_;
  int beam_count_;
  double angle_min_;
  double angle_max_;
  double range_min_;
  double range_max_;
  double scan_time_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudRingToLaserscan>());
  rclcpp::shutdown();
  return 0;
}
