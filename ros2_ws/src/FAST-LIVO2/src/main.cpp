#include "LIVMapper.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);

  // Sensor callbacks must keep draining DDS queues while the estimator is busy.
  rclcpp::executors::MultiThreadedExecutor callback_executor(
    rclcpp::ExecutorOptions(), 3);
  callback_executor.add_node(mapper.getNode());
  std::thread callback_thread([&callback_executor]() {
    callback_executor.spin();
  });

  RCLCPP_INFO(
    mapper.getNode()->get_logger(),
    "LiDAR, IMU, and image callbacks are running independently from state estimation");
  mapper.run(nh);

  callback_executor.cancel();
  if (callback_thread.joinable()) callback_thread.join();
  rclcpp::shutdown();
  return 0;
}
