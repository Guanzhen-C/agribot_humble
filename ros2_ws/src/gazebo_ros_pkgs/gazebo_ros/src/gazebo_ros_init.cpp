// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "gazebo_ros/gazebo_ros_init.hpp"

#include <memory>
#include <string>
#include <unordered_map>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorsIface.hh>

#include <gazebo_msgs/msg/performance_metrics.hpp>
#include <gazebo_msgs/msg/sensor_performance_metric.hpp>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>

#ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#if \
  (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION > 1) || \
  (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION > 14)
#define GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#endif
#endif  // ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS

namespace gazebo_ros
{

class GazeboRosInitPrivate
{
public:
  /// Constructor
  GazeboRosInitPrivate();

  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// Publish simulation time.
  /// \param[in] _info World update information.
  void PublishSimTime(const gazebo::common::UpdateInfo & _info);

  /// Callback from ROS service to reset simulation.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnResetSimulation(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to reset world.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnResetWorld(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to pause physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnPause(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to unpause (play) physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnUnpause(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  /// Publish Gazebo performance metrics directly to ROS.
  /// \param[in] info World update information.
  void PublishPerformanceMetrics(const gazebo::common::UpdateInfo & info);

  struct SensorPerformanceState
  {
    gazebo::common::Time last_measurement_time;
    gazebo::common::Time last_real_time;
    double sim_update_rate{0.0};
    double real_update_rate{0.0};
    double fps{-1.0};
    bool initialized{false};
    bool has_rates{false};
  };
#endif

  /// \brief Keep a pointer to the world.
  gazebo::physics::WorldPtr world_;

  /// Gazebo-ROS node
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publishes simulation time
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  /// ROS service to handle requests to reset simulation.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_simulation_service_;

  /// ROS service to handle requests to reset world.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_world_service_;

  /// ROS service to handle requests to pause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr unpause_service_;

  /// \brief ROS publisher to publish performance metrics.
  rclcpp::Publisher<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr performance_metrics_pub_;

  /// Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  /// Throttler for clock publisher.
  gazebo_ros::Throttler throttler_;

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  /// Per-sensor samples used to calculate actual update rates.
  std::unordered_map<std::string, SensorPerformanceState> sensor_performance_;

  /// Last times used to calculate real-time factor.
  gazebo::common::Time last_metrics_real_time_;
  gazebo::common::Time last_metrics_sim_time_;

  /// Publish performance metrics at Gazebo's original 5 Hz rate.
  gazebo_ros::Throttler performance_metrics_throttler_{5.0};
#endif

  /// Default frequency for clock publisher.
  static constexpr double DEFAULT_PUBLISH_FREQUENCY = 10.;
};

GazeboRosInit::GazeboRosInit()
: impl_(std::make_unique<GazeboRosInitPrivate>())
{
}

GazeboRosInit::~GazeboRosInit()
{
}

void GazeboRosInit::Load(int argc, char ** argv)
{
  // Initialize ROS with arguments
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
    impl_->ros_node_ = gazebo_ros::Node::Get();
  } else {
    impl_->ros_node_ = gazebo_ros::Node::Get();
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "gazebo_ros_init didn't initialize ROS "
      "because it's already initialized with other arguments");
  }

  if (gazebo_ros::ShouldDisplayEOLNotice()) {
    const char * msg =
      R"(
#     # ####### ####### ###  #####  #######
##    # #     #    #     #  #     # #
# #   # #     #    #     #  #       #
#  #  # #     #    #     #  #       #####
#   # # #     #    #     #  #       #
#    ## #     #    #     #  #     # #
#     # #######    #    ###  #####  #######

This version of Gazebo, now called Gazebo classic, reaches end-of-life
in January 2025. Users are highly encouraged to migrate to the new Gazebo
using our migration guides (https://gazebosim.org/docs/latest/gazebo_classic_migration?utm_source=gazebo_ros_pkgs&utm_medium=cli)

)";

    gzwarn << msg << std::endl;
  }

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  impl_->clock_pub_ = impl_->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::ClockQoS());

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  impl_->performance_metrics_pub_ =
    impl_->ros_node_->create_publisher<gazebo_msgs::msg::PerformanceMetrics>(
    "performance_metrics", 10);
#endif

  // Publish rate parameter
  auto rate_param = impl_->ros_node_->declare_parameter(
    "publish_rate",
    rclcpp::ParameterValue(GazeboRosInitPrivate::DEFAULT_PUBLISH_FREQUENCY));
  impl_->throttler_ = Throttler(rate_param.get<double>());

  // PerformanceMetrics parameter
  auto description_msg = rcl_interfaces::msg::ParameterDescriptor();
  description_msg.description =
    "If set to true, performance metrics are published to the topic /performance_metrics";

  impl_->ros_node_->declare_parameter<bool>("enable_performance_metrics", true, description_msg);

  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosInitPrivate::PublishSimTime, impl_.get(), std::placeholders::_1));

  // Get a callback when a world is created
  impl_->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosInitPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
void GazeboRosInitPrivate::PublishPerformanceMetrics(
  const gazebo::common::UpdateInfo & info)
{
  if (!world_ || performance_metrics_pub_->get_subscription_count() == 0) {
    return;
  }

  bool enabled = true;
  ros_node_->get_parameter("enable_performance_metrics", enabled);
  if (!enabled) {
    return;
  }

  const auto real_time = world_->RealTime();
  for (const auto & model : world_->Models()) {
    for (const auto & link : model->GetLinks()) {
      for (unsigned int index = 0; index < link->GetSensorCount(); ++index) {
        const auto name = link->GetSensorName(index);
        const auto sensor = gazebo::sensors::get_sensor(name);
        if (!sensor) {
          continue;
        }

        const auto measurement_time = sensor->LastMeasurementTime();
        auto & state = sensor_performance_[name];
        if (state.initialized && measurement_time < state.last_measurement_time) {
          state = SensorPerformanceState();
        }

        if (!state.initialized) {
          state.last_measurement_time = measurement_time;
          state.last_real_time = real_time;
          state.initialized = true;
          continue;
        }

        const auto sim_period = measurement_time - state.last_measurement_time;
        if (sim_period <= gazebo::common::Time::Zero) {
          continue;
        }

        const auto real_period = real_time - state.last_real_time;
        state.sim_update_rate = 1.0 / sim_period.Double();
        state.real_update_rate = real_period > gazebo::common::Time::Zero ?
          1.0 / real_period.Double() : 0.0;
        state.last_measurement_time = measurement_time;
        state.last_real_time = real_time;
        state.has_rates = true;

        const auto camera = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
        state.fps = camera && camera->Camera() ? camera->Camera()->AvgFPS() : -1.0;
      }
    }
  }

  if (!performance_metrics_throttler_.IsReady(info.simTime)) {
    return;
  }

  gazebo_msgs::msg::PerformanceMetrics metrics;
  metrics.header.stamp = Convert<builtin_interfaces::msg::Time>(info.simTime);

  const auto real_period = real_time - last_metrics_real_time_;
  const auto sim_period = info.simTime - last_metrics_sim_time_;
  metrics.real_time_factor = real_period > gazebo::common::Time::Zero &&
    sim_period >= gazebo::common::Time::Zero ?
    sim_period.Double() / real_period.Double() : 0.0;
  last_metrics_real_time_ = real_time;
  last_metrics_sim_time_ = info.simTime;

  for (const auto & item : sensor_performance_) {
    const auto & state = item.second;
    if (!state.has_rates) {
      continue;
    }

    gazebo_msgs::msg::SensorPerformanceMetric sensor_metric;
    sensor_metric.name = item.first;
    sensor_metric.sim_update_rate = state.sim_update_rate;
    sensor_metric.real_update_rate = state.real_update_rate;
    sensor_metric.fps = state.fps;
    metrics.sensors.push_back(sensor_metric);
  }

  performance_metrics_pub_->publish(metrics);
}
#endif

void GazeboRosInitPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_event_.reset();

  world_ = gazebo::physics::get_world(_world_name);

  // Reset services
  reset_simulation_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "reset_simulation",
    std::bind(
      &GazeboRosInitPrivate::OnResetSimulation, this,
      std::placeholders::_1, std::placeholders::_2));

  reset_world_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "reset_world",
    std::bind(
      &GazeboRosInitPrivate::OnResetWorld, this,
      std::placeholders::_1, std::placeholders::_2));

  // Pause services
  pause_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "pause_physics",
    std::bind(
      &GazeboRosInitPrivate::OnPause, this,
      std::placeholders::_1, std::placeholders::_2));

  unpause_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "unpause_physics",
    std::bind(
      &GazeboRosInitPrivate::OnUnpause, this,
      std::placeholders::_1, std::placeholders::_2));
}

GazeboRosInitPrivate::GazeboRosInitPrivate()
: throttler_(DEFAULT_PUBLISH_FREQUENCY)
{
}

void GazeboRosInitPrivate::PublishSimTime(const gazebo::common::UpdateInfo & _info)
{
#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  PublishPerformanceMetrics(_info);
#endif

  if (!throttler_.IsReady(_info.simTime)) {
    return;
  }

  rosgraph_msgs::msg::Clock clock;
  clock.clock = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  clock_pub_->publish(clock);
}

void GazeboRosInitPrivate::OnResetSimulation(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->Reset();
}

void GazeboRosInitPrivate::OnResetWorld(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->ResetEntities(gazebo::physics::Base::MODEL);
}

void GazeboRosInitPrivate::OnPause(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->SetPaused(true);
}

void GazeboRosInitPrivate::OnUnpause(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->SetPaused(false);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosInit)

}  // namespace gazebo_ros
