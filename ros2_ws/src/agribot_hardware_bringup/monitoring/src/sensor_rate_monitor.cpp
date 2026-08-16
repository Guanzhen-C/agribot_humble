#include <algorithm>
#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;

struct TopicSpec
{
  std::string source_topic;
  std::string type;
  std::string reported_topic;
};

struct RateWindow
{
  std::deque<SteadyClock::time_point> arrivals;
  SteadyClock::time_point last_arrival{};
  bool received{false};
};

diagnostic_msgs::msg::KeyValue keyValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string decimal(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}
}  // namespace

class SensorRateMonitor : public rclcpp::Node
{
public:
  SensorRateMonitor()
  : Node("sensor_rate_monitor")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/agribot/mobile_sensor_rates");
    window_sec_ = std::clamp(declare_parameter<double>("window_sec", 10.0), 2.0, 30.0);
    stale_sec_ = std::clamp(declare_parameter<double>("stale_sec", 2.0), 0.5, 10.0);

    const std::vector<TopicSpec> topics{
      {"/lidar/points", "sensor_msgs/msg/PointCloud2", "/lidar/points"},
      {"/imu/data", "sensor_msgs/msg/Imu", "/imu/data"},
      {"/camera/rgb/camera_info", "sensor_msgs/msg/CameraInfo", "/camera/rgb/image_raw"},
      {"/rtk/fix", "sensor_msgs/msg/NavSatFix", "/rtk/fix"},
      {"/wheel/odometry", "nav_msgs/msg/Odometry", "/wheel/odometry"},
    };

    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
    for (const auto & topic : topics) {
      sources_[topic.reported_topic] = topic.source_topic;
      windows_[topic.reported_topic] = RateWindow{};
      subscriptions_.push_back(create_generic_subscription(
        topic.source_topic,
        topic.type,
        qos,
        [this, reported_topic = topic.reported_topic](
          std::shared_ptr<rclcpp::SerializedMessage>) {
          record(reported_topic);
        }));
    }

    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output_topic_, 2);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishRates();});
    RCLCPP_INFO(
      get_logger(), "Lightweight sensor-rate monitor publishing on %s", output_topic_.c_str());
  }

private:
  void prune(RateWindow & window, const SteadyClock::time_point & now)
  {
    const auto oldest = now - std::chrono::duration<double>(window_sec_);
    while (!window.arrivals.empty() && window.arrivals.front() < oldest) {
      window.arrivals.pop_front();
    }
  }

  void record(const std::string & topic)
  {
    const auto now = SteadyClock::now();
    auto & window = windows_.at(topic);
    window.arrivals.push_back(now);
    window.last_arrival = now;
    window.received = true;
    prune(window, now);
  }

  void publishRates()
  {
    const auto now = SteadyClock::now();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = get_clock()->now();

    for (auto & item : windows_) {
      const auto & topic = item.first;
      auto & window = item.second;
      prune(window, now);

      double hz = 0.0;
      if (window.arrivals.size() >= 2) {
        const double elapsed = std::chrono::duration<double>(
          window.arrivals.back() - window.arrivals.front()).count();
        if (elapsed > 0.0) {
          hz = static_cast<double>(window.arrivals.size() - 1) / elapsed;
        }
      }
      const double age = window.received ?
        std::chrono::duration<double>(now - window.last_arrival).count() : -1.0;

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "agribot/mobile_sensor_rate" + topic;
      status.hardware_id = "rdk_x5";
      status.level = (!window.received || age > stale_sec_) ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = window.received ? "receiving" : "no data";
      status.values.push_back(keyValue("topic", topic));
      status.values.push_back(keyValue("source_topic", sources_.at(topic)));
      status.values.push_back(keyValue("hz", decimal(hz)));
      status.values.push_back(keyValue("age_sec", decimal(age)));
      message.status.push_back(std::move(status));
    }
    publisher_->publish(message);
  }

  std::string output_topic_;
  double window_sec_{10.0};
  double stale_sec_{2.0};
  std::unordered_map<std::string, std::string> sources_;
  std::unordered_map<std::string, RateWindow> windows_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorRateMonitor>());
  rclcpp::shutdown();
  return 0;
}
