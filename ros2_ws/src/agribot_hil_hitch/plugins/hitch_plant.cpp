#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "engagement.hpp"

namespace agribot_hitch {
class HitchPlant : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model; node_ = gazebo_ros::Node::Get(sdf);
    joint_ = model->GetJoint("hitch_lift_joint");
    tool_ = model->GetLink("hitch_tool_frame");
    if (!joint_ || !tool_) {throw std::runtime_error("Missing articulated hitch");}
    span_ = sdf->Get<double>("lift_span", .4).first;
    minimum_ = sdf->Get<double>("lift_min", 0.).first;
    zone_min_ = sdf->Get<ignition::math::Vector3d>("zone_min", {-30., -8., 0.}).first;
    zone_max_ = sdf->Get<ignition::math::Vector3d>("zone_max", {30., 8., 0.}).first;
    for (auto item = sdf->GetElement("disc"); item; item = item->GetNextElement("disc")) {
      Disc disc;
      disc.joint = model->GetJoint(item->Get<std::string>("joint"));
      disc.link = model->GetLink(item->Get<std::string>("link"));
      disc.radius = item->Get<double>("radius");
      if (!disc.joint || !disc.link) {throw std::runtime_error("Missing original CAD disc");}
      for (auto point = item->GetElement("probe"); point; point = point->GetNextElement("probe")) {
        disc.probes.push_back(point->Get<ignition::math::Vector3d>());
      }
      discs_.push_back(disc);
    }
    if (discs_.size() != 16 || span_ <= 0.) {throw std::runtime_error("Invalid hitch geometry");}
    if (sdf->HasElement("roller")) {
      auto item = sdf->GetElement("roller");
      roller_joint_ = model->GetJoint(item->Get<std::string>("joint"));
      roller_link_ = model->GetLink(item->Get<std::string>("link"));
      roller_radius_ = item->Get<double>("radius");
      if (!roller_joint_ || !roller_link_ || roller_radius_ <= 0.) {
        throw std::runtime_error("Missing original cage roller");
      }
    }
    command_ = node_->create_subscription<std_msgs::msg::Int16>(
      "/hil_hitch/actuator/valve", rclcpp::QoS(1),
      [this](std_msgs::msg::Int16::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        valve_ = std::abs(int(msg->data)) <= 1000 ? msg->data : 0;
        received_ = std::chrono::steady_clock::now();
      });
    state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/hil_hitch/joint_states", 10);
    robot_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    working_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hil_hitch/plant_state", rclcpp::QoS(1).best_effort());
    depth_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hil_hitch/disc_bottom_z", rclcpp::QoS(1).best_effort());
    transport_.reset(new gazebo::transport::Node()); transport_->Init(model->GetWorld()->Name());
    animation_ = transport_->Advertise<gazebo::msgs::Vector3d>("~/hil_hitch/tillage");
    trace_ = transport_->Advertise<gazebo::msgs::Pose>("~/hil_hitch/tillage_pose");
    linkage_ = transport_->Advertise<gazebo::msgs::Pose>("~/hil_hitch/lift_pose");
    swaths_ = transport_->Advertise<gazebo::msgs::PosesStamped>("~/hil_hitch/worked_swaths", 4);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &info) {Update(info);});
  }
  void Reset() override
  {
    swath_segments_.clear(); was_working_ = false;
    last_swath_publish_ = -1.;
  }
private:
  struct Disc {
    gazebo::physics::JointPtr joint;
    gazebo::physics::LinkPtr link;
    std::vector<ignition::math::Vector3d> probes;
    double radius;
  };
  void Update(const gazebo::common::UpdateInfo &info)
  {
    int valve;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      valve = valve_;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() > .2) {
        valve = 0;
      }
    }
    double q = joint_->Position(0), velocity = joint_->GetVelocity(0);
    if (!valve && was_flowing_) {locked_q_ = q;}
    was_flowing_ = valve != 0;
    const auto pivot = joint_->Anchor(0), axis = joint_->GlobalAxis(0);
    const auto gravity = model_->GetWorld()->Gravity();
    double gravity_torque = (tool_->WorldCoGPose().Pos()-pivot).Cross(
      tool_->GetInertial()->Mass()*gravity).Dot(axis);
    for (const auto &disc : discs_) {
      gravity_torque += (disc.link->WorldCoGPose().Pos()-pivot).Cross(
        disc.link->GetInertial()->Mass()*gravity).Dot(axis);
    }
    if (roller_link_) {
      gravity_torque += (roller_link_->WorldCoGPose().Pos()-pivot).Cross(
        roller_link_->GetInertial()->Mass()*gravity).Dot(axis);
    }
    // Equivalent hydraulic flow/viscosity and a closed-centre passive lock.
    // No target height reaches this plugin: the position controller is on 6750.
    double hydraulic = valve ? 14000.*(valve*.00012-velocity) :
      24000.*(locked_q_-q)-5000.*velocity;
    double torque = std::clamp(hydraulic-gravity_torque, -6000., 6000.);
    joint_->SetForce(0, torque);
    const auto pose = model_->WorldPose();
    double speed = pose.Rot().RotateVectorReverse(model_->WorldLinearVel()).X();
    double lowest = 1e6;
    std_msgs::msg::Float64MultiArray disc_bottom;
    ignition::math::Vector3d center;
    for (const auto &disc : discs_) {
      center += disc.link->WorldPose().Pos();
      double bottom = 1e6;
      for (const auto &point : disc.probes) {
        bottom = std::min(bottom, disc.link->WorldPose().CoordPositionAdd(point).Z());
      }
      disc_bottom.data.push_back(bottom); lowest = std::min(lowest, bottom);
    }
    center /= discs_.size();
    bool work = engaged(speed, lowest, center.X(), center.Y(),
      zone_min_.X(), zone_max_.X(), zone_min_.Y(), zone_max_.Y());
    const ignition::math::Pose3d worked_pose(center.X(), center.Y(), .015, 0, 0, pose.Rot().Yaw());
    const double now = info.simTime.Double();
    if (work) {
      const double distance = (worked_pose.Pos()-previous_work_.Pos()).Length();
      if (was_working_ && now-last_work_contact_ <= .4 && distance >= .20 && distance < 1.) {
        swath_segments_.emplace_back(previous_work_, worked_pose);
        if (swath_segments_.size() == 1) {gzmsg << "Harrow recorded first worked ground segment\n";}
        if (swath_segments_.size() > 512) {swath_segments_.pop_front();}
        previous_work_ = worked_pose;
      } else if (!was_working_ || now-last_work_contact_ > .4 || distance >= 1.) {previous_work_ = worked_pose;}
      was_working_ = true; last_work_contact_ = now;
    } else if (now-last_work_contact_ > .4 || lowest > .08) {
      was_working_ = false;
    }
    for (const auto &disc : discs_) {
      double target = work ? speed/disc.radius : 0.;
      disc.joint->SetForce(0, std::clamp(2.*(target-disc.joint->GetVelocity(0)), -20., 20.));
    }
    if (roller_joint_) {
      const bool contact = roller_link_->WorldPose().Pos().Z()-roller_radius_ < .035;
      const double target = contact ? speed/roller_radius_ : 0.;
      roller_joint_->SetForce(0, std::clamp(4.*(target-roller_joint_->GetVelocity(0)), -20., 20.));
    }
    if (now < last_swath_publish_ || now-last_swath_publish_ >= .5) {
      gazebo::msgs::PosesStamped history;
      gazebo::msgs::Set(history.mutable_time(), info.simTime);
      for (const auto &segment : swath_segments_) {
        gazebo::msgs::Set(history.add_pose(), segment.first);
        gazebo::msgs::Set(history.add_pose(), segment.second);
      }
      swaths_->Publish(history); last_swath_publish_ = now;
    }
    if (now < last_time_ || now-last_time_ >= .05) {
      sensor_msgs::msg::JointState state;
      state.header.stamp = node_->get_clock()->now(); state.name.push_back("hitch_lift_joint");
      state.position.push_back(q); state.velocity.push_back(velocity); state.effort.push_back(torque);
      for (const auto &disc : discs_) {
        state.name.push_back(disc.joint->GetName()); state.position.push_back(disc.joint->Position(0));
        state.velocity.push_back(disc.joint->GetVelocity(0)); state.effort.push_back(0.);
      }
      if (roller_joint_) {
        state.name.push_back(roller_joint_->GetName()); state.position.push_back(roller_joint_->Position(0));
        state.velocity.push_back(roller_joint_->GetVelocity(0)); state.effort.push_back(0.);
      }
      state_->publish(state);
      robot_state_->publish(state);
      std_msgs::msg::Float64MultiArray status;
      status.data = {std::clamp((q-minimum_)/span_, 0., 1.)*10000., velocity/span_*10000.,
        static_cast<double>(work), lowest, speed, static_cast<double>(valve),
        center.X(), center.Y(), now};
      working_->publish(status);
      depth_->publish(disc_bottom);
      gazebo::msgs::Vector3d activity; activity.set_x(work); activity.set_y(std::abs(speed)); activity.set_z(now);
      animation_->Publish(activity);
      gazebo::msgs::Pose trail;
      gazebo::msgs::Set(&trail, ignition::math::Pose3d(center.X(), center.Y(), .015, 0, 0, pose.Rot().Yaw()));
      trace_->Publish(trail);
      gazebo::msgs::Pose relative_pose;
      gazebo::msgs::Set(&relative_pose, model_->GetLink("base_link")->WorldPose().Inverse()*tool_->WorldPose());
      linkage_->Publish(relative_pose);
      last_time_ = now;
    }
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr tool_;
  gazebo::physics::JointPtr roller_joint_;
  gazebo::physics::LinkPtr roller_link_;
  double roller_radius_{.268};
  std::vector<Disc> discs_;
  gazebo_ros::Node::SharedPtr node_;
  ignition::math::Vector3d zone_min_, zone_max_;
  double span_{.4}, minimum_{0.}, locked_q_{0.}, last_time_{-1.};
  bool was_flowing_{true};
  bool was_working_{false};
  double last_swath_publish_{-1.}, last_work_contact_{-1.};
  ignition::math::Pose3d previous_work_;
  std::deque<std::pair<ignition::math::Pose3d, ignition::math::Pose3d>> swath_segments_;
  int valve_{0};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  gazebo::event::ConnectionPtr update_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr animation_, trace_, linkage_, swaths_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr working_, depth_;
};
GZ_REGISTER_MODEL_PLUGIN(HitchPlant)
}
