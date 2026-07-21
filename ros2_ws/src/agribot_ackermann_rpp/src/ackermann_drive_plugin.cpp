#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ignition/math/Vector3.hh>

namespace agribot_ackermann_rpp
{

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double wrap_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

class AckermannDrivePlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    sdf_ = std::move(sdf);

    ros_node_ = gazebo_ros::Node::Get(sdf_);

    wheelbase_ = sdf_->Get<double>("wheelbase", 0.498).first;
    track_width_ = sdf_->Get<double>("track_width", 0.58306).first;
    wheel_radius_ = sdf_->Get<double>("wheel_radius", 0.16459).first;
    max_steering_angle_ = sdf_->Get<double>("max_steering_angle", 0.6).first;
    max_steering_rate_ = sdf_->Get<double>("max_steering_rate", 1.2).first;
    max_speed_ = sdf_->Get<double>("max_speed", 1.0).first;
    max_accel_ = sdf_->Get<double>("max_accel", 1.5).first;
    min_speed_for_curvature_ = sdf_->Get<double>("min_speed_for_curvature", 0.05).first;
    command_timeout_ = sdf_->Get<double>("command_timeout", 1.5).first;
    publish_rate_ = sdf_->Get<double>("publish_rate", 30.0).first;
    publish_tf_ = sdf_->Get<bool>("publish_tf", true).first;
    wheel_torque_ = sdf_->Get<double>("wheel_torque", 200.0).first;
    sensor_model_name_ = sdf_->Get<std::string>("sensor_model_name", "").first;
    odom_topic_ = sdf_->Get<std::string>("odom_topic", "/odom").first;
    joint_state_topic_ = sdf_->Get<std::string>("joint_state_topic", "/joint_states").first;
    cmd_vel_topic_ = sdf_->Get<std::string>("cmd_vel_topic", "/nav2/cmd_vel").first;
    odom_frame_ = sdf_->Get<std::string>("odom_frame", "odom").first;
    base_frame_ = sdf_->Get<std::string>("base_frame", "base_link").first;

    front_left_steering_joint_ = model_->GetJoint("front_left_steering_joint");
    front_right_steering_joint_ = model_->GetJoint("front_right_steering_joint");
    front_left_wheel_joint_ = model_->GetJoint("front_left_wheel_joint");
    front_right_wheel_joint_ = model_->GetJoint("front_right_wheel_joint");
    rear_left_wheel_joint_ = model_->GetJoint("rear_left_wheel_joint");
    rear_right_wheel_joint_ = model_->GetJoint("rear_right_wheel_joint");
    base_link_ = model_->GetLink("base_link");

    if (
      !front_left_steering_joint_ || !front_right_steering_joint_ ||
      !front_left_wheel_joint_ || !front_right_wheel_joint_ ||
      !rear_left_wheel_joint_ || !rear_right_wheel_joint_ || !base_link_)
    {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "AckermannDrivePlugin could not find the base link or one or more required joints");
      return;
    }

    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&AckermannDrivePlugin::OnCmdVel, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);
    }

    const auto pose = model_->WorldPose();
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    z_ = pose.Pos().Z();
    yaw_ = pose.Rot().Yaw();

    last_cmd_time_ = ros_node_->now();
    last_publish_time_ = model_->GetWorld()->SimTime();
    last_update_time_ = model_->GetWorld()->SimTime();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannDrivePlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "AckermannDrivePlugin loaded for model [%s], subscribing to [%s], using wheel joint drive",
      model_->GetName().c_str(), cmd_vel_topic_.c_str());
  }

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    target_linear_ = clamp(msg->linear.x, -max_speed_, max_speed_);
    target_angular_ = msg->angular.z;
    last_cmd_time_ = ros_node_->now();
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const auto sim_time = info.simTime;
    const double dt = (sim_time - last_update_time_).Double();
    last_update_time_ = sim_time;
    if (dt <= 0.0) {
      return;
    }

    double target_linear = 0.0;
    double target_angular = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if ((ros_node_->now() - last_cmd_time_).seconds() <= command_timeout_) {
        target_linear = target_linear_;
        target_angular = target_angular_;
      }
    }

    current_linear_ = approach(current_linear_, target_linear, max_accel_ * dt);
    const double steering_target = compute_steering_target(target_angular, target_linear);
    current_steering_ = approach(
      current_steering_, steering_target, max_steering_rate_ * dt);

    if (std::abs(current_linear_) < 1e-4) {
      current_angular_ = 0.0;
    } else {
      current_angular_ = current_linear_ * std::tan(current_steering_) / wheelbase_;
    }

    const auto steering_angles = split_steering_angles(current_steering_);
    apply_joint_control(steering_angles.first, steering_angles.second);

    if ((sim_time - last_publish_time_).Double() >= (1.0 / std::max(1.0, publish_rate_))) {
      publish_ros(sim_time, steering_angles.first, steering_angles.second);
      last_publish_time_ = sim_time;
    }
  }

  double approach(const double current, const double target, const double max_delta) const
  {
    const double delta = target - current;
    if (delta > max_delta) {
      return current + max_delta;
    }
    if (delta < -max_delta) {
      return current - max_delta;
    }
    return target;
  }

  double compute_steering_target(const double angular, const double linear) const
  {
    if (std::abs(linear) < min_speed_for_curvature_ || std::abs(angular) < 1e-5) {
      return 0.0;
    }
    const double steering = std::atan(wheelbase_ * angular / linear);
    return clamp(steering, -max_steering_angle_, max_steering_angle_);
  }

  std::pair<double, double> split_steering_angles(const double center_angle) const
  {
    if (std::abs(center_angle) < 1e-5) {
      return {0.0, 0.0};
    }

    const bool turn_left = center_angle > 0.0;
    const double radius = std::abs(wheelbase_ / std::tan(center_angle));
    const double inner_radius = std::max(radius - track_width_ * 0.5, 1e-4);
    const double outer_radius = radius + track_width_ * 0.5;
    const double inner_angle = std::atan(wheelbase_ / inner_radius);
    const double outer_angle = std::atan(wheelbase_ / outer_radius);

    if (turn_left) {
      return {inner_angle, outer_angle};
    }
    return {-outer_angle, -inner_angle};
  }

  void update_actual_state()
  {
    const auto pose = model_->WorldPose();
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    z_ = pose.Pos().Z();
    yaw_ = pose.Rot().Yaw();
  }

  double forward_velocity() const
  {
    const auto velocity = base_link_->WorldLinearVel();
    return velocity.X() * std::cos(yaw_) + velocity.Y() * std::sin(yaw_);
  }

  void set_wheel_drive(
    const gazebo::physics::JointPtr & joint,
    const double angular_velocity) const
  {
    joint->SetParam("fmax", 0, wheel_torque_);
    joint->SetParam("vel", 0, angular_velocity);
  }

  void update_wheel_joint_positions()
  {
    front_left_wheel_position_ = front_left_wheel_joint_->Position(0);
    front_right_wheel_position_ = front_right_wheel_joint_->Position(0);
    rear_left_wheel_position_ = rear_left_wheel_joint_->Position(0);
    rear_right_wheel_position_ = rear_right_wheel_joint_->Position(0);
  }

  void update_sensor_model_pose()
  {
    if (!sensor_model_name_.empty()) {
      if (!sensor_model_) {
        sensor_model_ = model_->GetWorld()->ModelByName(sensor_model_name_);
      }
      if (sensor_model_) {
        const auto pose = model_->WorldPose();
        const auto actual_velocity = base_link_->WorldLinearVel();
        sensor_model_->SetWorldPose(pose, true, true);
        sensor_model_->SetLinearVel(
          ignition::math::Vector3d(actual_velocity.X(), actual_velocity.Y(), actual_velocity.Z()));
        sensor_model_->SetAngularVel(base_link_->WorldAngularVel());
      }
    }
  }

  void apply_joint_control(const double left_steer, const double right_steer)
  {
    const double safe_radius = std::max(wheel_radius_, 1e-6);
    const double left_linear = current_linear_ - current_angular_ * track_width_ * 0.5;
    const double right_linear = current_linear_ + current_angular_ * track_width_ * 0.5;
    const double left_wheel_velocity = left_linear / safe_radius;
    const double right_wheel_velocity = right_linear / safe_radius;

    front_left_steering_joint_->SetPosition(0, left_steer, true);
    front_right_steering_joint_->SetPosition(0, right_steer, true);
    set_wheel_drive(front_left_wheel_joint_, left_wheel_velocity);
    set_wheel_drive(rear_left_wheel_joint_, left_wheel_velocity);
    set_wheel_drive(front_right_wheel_joint_, right_wheel_velocity);
    set_wheel_drive(rear_right_wheel_joint_, right_wheel_velocity);
    update_wheel_joint_positions();
    update_sensor_model_pose();
  }

  void publish_ros(
    const gazebo::common::Time & sim_time,
    const double left_steer,
    const double right_steer)
  {
    update_actual_state();
    const double actual_forward = forward_velocity();
    const double actual_yaw_rate = base_link_->WorldAngularVel().Z();
    const rclcpp::Time stamp(sim_time.sec, sim_time.nsec);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = quaternion_from_yaw(yaw_);
    odom.twist.twist.linear.x = actual_forward;
    odom.twist.twist.angular.z = actual_yaw_rate;
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.02;
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.02;
    odom_pub_->publish(odom);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = {
      "front_left_steering_joint",
      "front_right_steering_joint",
      "front_left_wheel_joint",
      "front_right_wheel_joint",
      "rear_left_wheel_joint",
      "rear_right_wheel_joint"
    };
    joint_state.position = {
      left_steer,
      right_steer,
      front_left_wheel_position_,
      front_right_wheel_position_,
      rear_left_wheel_position_,
      rear_right_wheel_position_
    };
    joint_pub_->publish(joint_state);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = quaternion_from_yaw(yaw_);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  gazebo::physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::event::ConnectionPtr update_connection_;

  gazebo::physics::JointPtr front_left_steering_joint_;
  gazebo::physics::JointPtr front_right_steering_joint_;
  gazebo::physics::JointPtr front_left_wheel_joint_;
  gazebo::physics::JointPtr front_right_wheel_joint_;
  gazebo::physics::JointPtr rear_left_wheel_joint_;
  gazebo::physics::JointPtr rear_right_wheel_joint_;
  gazebo::physics::LinkPtr base_link_;
  gazebo::physics::ModelPtr sensor_model_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex command_mutex_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  gazebo::common::Time last_update_time_;
  gazebo::common::Time last_publish_time_;

  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double wheelbase_{0.498};
  double track_width_{0.58306};
  double wheel_radius_{0.16459};
  double max_steering_angle_{0.6};
  double max_steering_rate_{1.2};
  double max_speed_{1.0};
  double max_accel_{1.5};
  double min_speed_for_curvature_{0.05};
  double command_timeout_{1.5};
  double publish_rate_{30.0};
  double wheel_torque_{200.0};
  bool publish_tf_{true};
  std::string sensor_model_name_;

  double x_{0.0};
  double y_{0.0};
  double z_{0.24};
  double yaw_{0.0};
  double target_linear_{0.0};
  double target_angular_{0.0};
  double current_linear_{0.0};
  double current_angular_{0.0};
  double current_steering_{0.0};
  double front_left_wheel_position_{0.0};
  double front_right_wheel_position_{0.0};
  double rear_left_wheel_position_{0.0};
  double rear_right_wheel_position_{0.0};
};

}  // namespace agribot_ackermann_rpp

GZ_REGISTER_MODEL_PLUGIN(agribot_ackermann_rpp::AckermannDrivePlugin)
