#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace agribot_hardware_bringup
{

class PointCloudTfAdapter : public rclcpp::Node
{
public:
  PointCloudTfAdapter()
  : Node("pointcloud_tf_adapter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/lidar_points_base");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.10);

    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty()) {
      throw std::runtime_error("point cloud topics and target_frame must not be empty");
    }
    if (transform_timeout_ < 0.0) {
      throw std::runtime_error("transform_timeout must be non-negative");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudTfAdapter::handle_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Transforming %s into %s on %s",
      input_topic_.c_str(), target_frame_.c_str(), output_topic_.c_str());
  }

private:
  void handle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud with empty frame_id");
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id, message->header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_));
      sensor_msgs::msg::PointCloud2 output;
      tf2::doTransform(*message, output, transform);
      output.header.stamp = message->header.stamp;
      output.header.frame_id = target_frame_;
      publisher_->publish(output);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Point cloud transform failed: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_{0.10};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agribot_hardware_bringup::PointCloudTfAdapter>());
  rclcpp::shutdown();
  return 0;
}
