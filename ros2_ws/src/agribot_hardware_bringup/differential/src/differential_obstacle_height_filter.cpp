#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DifferentialObstacleHeightFilter : public rclcpp::Node
{
public:
  DifferentialObstacleHeightFilter()
  : Node("differential_obstacle_height_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/navigation/obstacle_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    min_height_ = declare_parameter<double>("min_height", 0.90);
    max_height_ = declare_parameter<double>("max_height", 1.00);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

    if (input_topic_.empty() || output_topic_.empty() || base_frame_.empty()) {
      throw std::invalid_argument("point-cloud topics and base_frame must not be empty");
    }
    if (!std::isfinite(min_height_) || !std::isfinite(max_height_) ||
      min_height_ > max_height_)
    {
      throw std::invalid_argument("min_height must be finite and no greater than max_height");
    }
    if (!std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ < 0.0) {
      throw std::invalid_argument("transform_timeout_sec must be finite and non-negative");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(1));
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&DifferentialObstacleHeightFilter::handle_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Filtering %s in %s height band [%.2f, %.2f] m -> %s",
      input_topic_.c_str(), base_frame_.c_str(), min_height_, max_height_,
      output_topic_.c_str());
  }

private:
  void handle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud with an empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot transform %s to %s: %s",
        message->header.frame_id.c_str(), base_frame_.c_str(), error.what());
      return;
    }

    const auto & rotation_message = transform.transform.rotation;
    Eigen::Quaterniond quaternion(
      rotation_message.w, rotation_message.x, rotation_message.y, rotation_message.z);
    if (quaternion.norm() < 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud due to invalid TF rotation");
      return;
    }
    quaternion.normalize();
    const Eigen::RowVector3d base_z_axis = quaternion.toRotationMatrix().row(2);
    const double base_z_offset = transform.transform.translation.z;

    std::vector<std::array<float, 3>> selected;
    selected.reserve(message->width * message->height / 8U);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const double height =
          base_z_axis.x() * static_cast<double>(*x) +
          base_z_axis.y() * static_cast<double>(*y) +
          base_z_axis.z() * static_cast<double>(*z) + base_z_offset;
        if (height >= min_height_ && height <= max_height_) {
          selected.push_back({*x, *y, *z});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Invalid PointCloud2 layout: %s", error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = message->header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(selected.size());
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    for (const auto & point : selected) {
      *out_x = point[0];
      *out_y = point[1];
      *out_z = point[2];
      ++out_x;
      ++out_y;
      ++out_z;
    }
    output.is_dense = true;
    publisher_->publish(std::move(output));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  double min_height_{0.90};
  double max_height_{1.00};
  double transform_timeout_sec_{0.05};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialObstacleHeightFilter>());
  rclcpp::shutdown();
  return 0;
}
