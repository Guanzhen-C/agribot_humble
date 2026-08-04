#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace agribot_offline_mapping
{
namespace
{

struct Point
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  float time;
};

const sensor_msgs::msg::PointField & fieldByName(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
  if (iterator == cloud.fields.end()) {
    throw std::runtime_error("input point cloud has no '" + name + "' field");
  }
  if (iterator->count != 1U) {
    throw std::runtime_error("input point field '" + name + "' must be scalar");
  }
  return *iterator;
}

template<typename Value>
Value readValue(const std::uint8_t * point, const sensor_msgs::msg::PointField & field)
{
  Value result{};
  std::memcpy(&result, point + field.offset, sizeof(Value));
  return result;
}

std::size_t datatypeSize(std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1U;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2U;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4U;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8U;
    default:
      throw std::runtime_error("input point cloud contains an unknown field datatype");
  }
}

double readTime(const std::uint8_t * point, const sensor_msgs::msg::PointField & field)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return static_cast<double>(readValue<float>(point, field));
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return readValue<double>(point, field);
  }
  throw std::runtime_error("input point field 'time' must be FLOAT32 or FLOAT64");
}

}  // namespace

class LslidarLioSamAdapter final : public rclcpp::Node
{
public:
  LslidarLioSamAdapter()
  : Node("lslidar_lio_sam_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lio_sam/points");
    output_frame_ = declare_parameter<std::string>("output_frame", "lidar_link");
    maximum_scan_duration_ = declare_parameter<double>("maximum_scan_duration", 0.20);
    maximum_ring_ = declare_parameter<int>("maximum_ring", 15);
    if (maximum_scan_duration_ <= 0.0 || maximum_ring_ < 0 || maximum_ring_ > 65535) {
      throw std::runtime_error("invalid scan duration or ring limit");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LslidarLioSamAdapter::handleCloud, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Normalizing C16 ring/time fields for LIO-SAM: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void handleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    try {
      publisher_->publish(convert(*message));
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting incompatible C16 cloud: %s", error.what());
    }
  }

  sensor_msgs::msg::PointCloud2 convert(const sensor_msgs::msg::PointCloud2 & input) const
  {
    if (input.is_bigendian) {
      throw std::runtime_error("big-endian PointCloud2 input is not supported");
    }
    const auto & x_field = fieldByName(input, "x");
    const auto & y_field = fieldByName(input, "y");
    const auto & z_field = fieldByName(input, "z");
    const auto & intensity_field = fieldByName(input, "intensity");
    const auto & ring_field = fieldByName(input, "ring");
    const auto & time_field = fieldByName(input, "time");
    for (const auto * field :
      {&x_field, &y_field, &z_field, &intensity_field, &ring_field, &time_field})
    {
      if (static_cast<std::size_t>(field->offset) + datatypeSize(field->datatype) >
        input.point_step)
      {
        throw std::runtime_error("input point field exceeds point_step");
      }
    }
    for (const auto * field : {&x_field, &y_field, &z_field, &intensity_field}) {
      if (field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
        throw std::runtime_error("x/y/z/intensity fields must be FLOAT32");
      }
    }
    if (ring_field.datatype != sensor_msgs::msg::PointField::UINT16) {
      throw std::runtime_error("ring field must be UINT16");
    }
    if (input.point_step == 0U || input.width == 0U || input.height == 0U ||
      input.row_step < input.width * input.point_step ||
      input.data.size() < static_cast<std::size_t>(input.row_step) * input.height)
    {
      throw std::runtime_error("PointCloud2 dimensions are inconsistent");
    }

    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(input.width) * input.height);
    double minimum_time = std::numeric_limits<double>::infinity();
    double maximum_time = -std::numeric_limits<double>::infinity();
    for (std::uint32_t row = 0U; row < input.height; ++row) {
      const std::uint8_t * row_data = input.data.data() + row * input.row_step;
      for (std::uint32_t column = 0U; column < input.width; ++column) {
        const std::uint8_t * raw = row_data + column * input.point_step;
        Point point{
          readValue<float>(raw, x_field), readValue<float>(raw, y_field),
          readValue<float>(raw, z_field), readValue<float>(raw, intensity_field),
          readValue<std::uint16_t>(raw, ring_field), 0.0F};
        const double point_time = readTime(raw, time_field);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
          !std::isfinite(point_time))
        {
          continue;
        }
        if (point.ring > static_cast<std::uint16_t>(maximum_ring_)) {
          throw std::runtime_error("point ring exceeds configured C16 ring limit");
        }
        point.time = static_cast<float>(point_time);
        minimum_time = std::min(minimum_time, point_time);
        maximum_time = std::max(maximum_time, point_time);
        points.push_back(point);
      }
    }
    if (points.empty()) {
      throw std::runtime_error("point cloud contains no finite points");
    }
    const double duration = maximum_time - minimum_time;
    if (!std::isfinite(duration) || duration < 0.0 || duration > maximum_scan_duration_ ||
      std::abs(minimum_time) > 10.0 * maximum_scan_duration_ ||
      std::abs(maximum_time) > 10.0 * maximum_scan_duration_)
    {
      throw std::runtime_error("point times are not relative times for one scan");
    }
    for (Point & point : points) {
      point.time = static_cast<float>(static_cast<double>(point.time) - minimum_time);
    }
    std::stable_sort(
      points.begin(), points.end(),
      [](const Point & left, const Point & right) {return left.time < right.time;});

    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.header.frame_id = output_frame_;
    output.header.stamp =
      rclcpp::Time(input.header.stamp) + rclcpp::Duration::from_seconds(minimum_time);
    output.height = 1U;
    output.width = static_cast<std::uint32_t>(points.size());
    output.is_bigendian = false;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> ring(output, "ring");
    sensor_msgs::PointCloud2Iterator<float> time(output, "time");
    for (const Point & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *intensity = point.intensity;
      *ring = point.ring;
      *time = point.time;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++ring;
      ++time;
    }
    return output;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double maximum_scan_duration_{0.20};
  int maximum_ring_{15};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace agribot_offline_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agribot_offline_mapping::LslidarLioSamAdapter>());
  rclcpp::shutdown();
  return 0;
}
