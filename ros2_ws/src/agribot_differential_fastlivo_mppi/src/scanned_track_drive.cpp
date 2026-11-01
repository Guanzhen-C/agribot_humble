#include <atomic>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <unistd.h>

#include <gazebo/common/Events.hh>
#include <gazebo/ode/ode.h>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/plugins/SimpleTrackedVehiclePlugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "track_command_limits.hpp"
#include "scanned_collision_layers.hpp"

namespace agribot_differential_fastlivo_mppi
{
// Transport and actuator limits only; native ODE contacts supply all traction.
class ScannedTrackDrive : public gazebo::SimpleTrackedVehiclePlugin
{
public:
  ~ScannedTrackDrive() override
  {
    update_.reset();
    subscription_.reset();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    gazebo::SimpleTrackedVehiclePlugin::Load(model, sdf);
    world_ = model->GetWorld();
    body_ = model->GetLink("base_link");
    track_half_width_ = sdf->Get<double>("tracks_separation") / 2;
    efficiency_ = sdf->Get<double>("steering_efficiency");
    max_track_speed_ = sdf->Get<double>("max_linear_speed");
    linear_acceleration_ = sdf->Get<double>("max_linear_acceleration", .5).first;
    angular_acceleration_ = sdf->Get<double>("max_angular_acceleration", .8).first;
    node_ = gazebo_ros::Node::Get(sdf);
    subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/nav2/cmd_vel", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
          linear_.store(0.0);
          angular_.store(0.0);
        } else {
          linear_.store(message->linear.x);
          angular_.store(message->angular.z);
        }
        sequence_.fetch_add(1);
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScannedTrackDrive::UpdateCommand, this));
    RCLCPP_INFO(node_->get_logger(), "Continuous ODE track traction on /nav2/cmd_vel; no pose or velocity-state override");
  }

  void Reset() override
  {
    linear_.store(0.0);
    angular_.store(0.0);
    sequence_.fetch_add(1);
    applied_command_ = {};
    previous_update_ = world_->SimTime().Double();
    gazebo::SimpleTrackedVehiclePlugin::Reset();
  }

private:
  bool ConfigureQueryLayers()
  {
    const auto support = world_->ModelByName("reconstructed_ground_support");
    if (!support) {
      return true;
    }
    unsigned int ray_count = 0;
    for (const auto & sensor : gazebo::sensors::SensorManager::Instance()->GetSensors()) {
      const auto ray_sensor = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
      if (!ray_sensor || sensor->ParentName().find(body_->GetModel()->GetScopedName()+"::") != 0) {
        continue;
      }
      const auto shape = ray_sensor->LaserShape();
      if (!shape || !shape->RayCount()) {
        continue;
      }
      for (unsigned int i = 0; i < shape->RayCount(); ++i) {
        const auto ray = boost::dynamic_pointer_cast<gazebo::physics::ODERayShape>(shape->Ray(i));
        if (!ray) {
          continue;
        }
        const auto geometry = ray->ODEGeomId();
        dGeomSetCategoryBits(geometry, kScannedRayCategory);
        dGeomSetCollideBits(geometry, kScannedRayMask);
        // MultiRay has nested spaces with independent broad-phase masks.
        const auto space = dGeomGetSpace(geometry);
        dGeomSetCategoryBits(reinterpret_cast<dGeomID>(space), kScannedRayCategory);
        dGeomSetCollideBits(reinterpret_cast<dGeomID>(space), kScannedRayMask);
        ++ray_count;
      }
    }
    if (!ray_count) {
      return false;
    }
    // Full source meshes remain the sensor scene. Derived heightfield and
    // obstacle proxies supply contact only, not duplicate ray intersections.
    for (const auto & model : {support, world_->ModelByName("k2_automation_sensor_scene")}) {
      if (!model) {
        continue;
      }
      for (const auto & link : model->GetLinks()) {
        for (const auto & collision : link->GetCollisions()) {
          if (model == support) {
            collision->SetCategoryBits(kScannedRayCategory);
            collision->SetCollideBits(kScannedPhysicalMask);
            const auto ode = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision);
            if (ode && collision->GetShape()->HasType(gazebo::physics::Base::HEIGHTMAP_SHAPE)) {
              // Gazebo 11's ODEHeightmapShape applies the orientation but
              // leaves the static heightfield translation at the origin.
              // Place only this static terrain collider at its SDF pose.
              const auto position = collision->WorldPose().Pos();
              dGeomSetPosition(ode->GetCollisionId(), position.X(), position.Y(), position.Z());
            }
          } else if (collision->GetName().find("_hard_contact") != std::string::npos) {
            const auto ode = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision);
            if (ode) {
              // Do not mask the shared parent link: it also contains the
              // unmodified source mesh that LiDAR must continue to observe.
              dGeomSetCategoryBits(ode->GetCollisionId(), kScannedRayCategory);
              dGeomSetCollideBits(ode->GetCollisionId(), kScannedPhysicalMask);
            }
          }
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "Separated %u source-scene rays from derived terrain contact", ray_count);
    return true;
  }

  void UpdateCommand()
  {
    if (!query_layers_configured_) {
      query_layers_configured_ = ConfigureQueryLayers();
    }
    const double now = world_->SimTime().Double();
    const auto velocity = body_->WorldLinearVel();
    if (!snapshot_taken_ && std::hypot(velocity.X(), velocity.Y()) > 1.0) {
      snapshot_taken_ = true;
      const auto file = "/tmp/agribot_scanned_track_instability_" + std::to_string(getpid()) + ".txt";
      std::ofstream output(file);
      output << "sim_time " << now << "\npose " << body_->WorldPose()
             << "\nvelocity " << velocity << "\nangular " << body_->WorldAngularVel()
             << "\napplied_command " << applied_command_.linear << " " << applied_command_.angular << "\n";
      const auto manager = world_->Physics()->GetContactManager();
      const auto & contacts = manager->GetContacts();
      for (unsigned int i = 0; i < manager->GetContactCount(); ++i) {
        if (contacts[i]->collision1->GetModel() == body_->GetModel() ||
          contacts[i]->collision2->GetModel() == body_->GetModel())
        {
          output << contacts[i]->DebugString() << "\n";
        }
      }
      RCLCPP_WARN(node_->get_logger(), "Unexpected track motion; contact snapshot: %s", file.c_str());
    }
    const auto sequence = sequence_.load();
    if (sequence != applied_sequence_) {
      applied_sequence_ = sequence;
      last_command_ = now;
    }
    const bool fresh = now >= last_command_ && now - last_command_ <= 0.6;
    const double linear = fresh ? linear_.load() : 0.0;
    const double angular = fresh ? angular_.load() : 0.0;
    applied_command_ = limitedTrackCommand(
      applied_command_, {linear, angular}, now - previous_update_,
      linear_acceleration_, angular_acceleration_,
      track_half_width_, efficiency_, max_track_speed_);
    previous_update_ = now;
    // Native tracked control uses the opposite angular convention to ROS FLU.
    SetBodyVelocity(applied_command_.linear, -applied_command_.angular);
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr body_;
  bool snapshot_taken_{false};
  bool query_layers_configured_{false};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  gazebo::event::ConnectionPtr update_;
  std::atomic<double> linear_{0.0}, angular_{0.0};
  std::atomic<unsigned long> sequence_{0};
  unsigned long applied_sequence_{0};
  double last_command_{-1.0};
  double previous_update_{0.0};
  TrackCommand applied_command_;
  double linear_acceleration_{.5}, angular_acceleration_{.8};
  double track_half_width_{0.3}, efficiency_{0.36}, max_track_speed_{0.78};
};
GZ_REGISTER_MODEL_PLUGIN(ScannedTrackDrive)
}  // namespace agribot_differential_fastlivo_mppi
