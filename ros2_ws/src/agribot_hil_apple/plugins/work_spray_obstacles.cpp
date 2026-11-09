#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "spray_occlusion.hpp"

namespace agribot_hil
{
class WorkSprayObstacles : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    link_ = model_->GetLink(sdf->Get<std::string>("link_name", "work_plant_frame").first);
    if (!link_) {throw std::runtime_error("Missing spray collision reference link");}
    ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      model_->GetWorld()->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    for (auto node = sdf->GetElement("nozzle"); node; node = node->GetNextElement("nozzle")) {
      SprayDepthGrid grid;
      grid.origin = node->Get<ignition::math::Vector3d>("position");
      grid.direction = node->Get<ignition::math::Vector3d>("direction").Normalized();
      grid.u = grid.direction.Cross(std::abs(grid.direction.Z()) < .95 ?
        ignition::math::Vector3d::UnitZ : ignition::math::Vector3d::UnitY).Normalized();
      grid.v = grid.direction.Cross(grid.u).Normalized();
      nozzles_.push_back(grid);
    }
    node_.reset(new gazebo::transport::Node());
    node_->Init(model_->GetWorld()->Name());
    publisher_ = node_->Advertise<gazebo::msgs::GzString>("~/hil/work/spray_depth");
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & info) {Update(info);});
  }

private:
  double Distance(const ignition::math::Vector3d & origin,
                  const ignition::math::Vector3d & direction)
  {
    constexpr double maximum = 4.5;
    double advanced = 0.;
    // The broad chassis proxy extends above the deck. Ignore only this model;
    // do not replace real environment meshes by their oversized bounding boxes.
    for (unsigned skip = 0; skip < 8 && advanced < maximum; ++skip) {
      const auto start = origin+direction*advanced;
      ray_->SetPoints(start, origin+direction*maximum);
      double distance = maximum-advanced;
      std::string entity;
      ray_->GetIntersection(distance, entity);
      if (entity.empty()) {return maximum;}
      if (entity.find(model_->GetScopedName()+"::") != 0) {
        return std::clamp(advanced+distance, 0., maximum);
      }
      auto collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(
        model_->GetWorld()->EntityByName(entity));
      if (!collision) {return 0.;}
      const auto bounds = collision->BoundingBox();
      double exit = std::numeric_limits<double>::infinity();
      for (unsigned axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) > 1e-9) {
          const double edge = direction[axis] > 0. ? bounds.Max()[axis] : bounds.Min()[axis];
          const double value = (edge-origin[axis])/direction[axis];
          if (value > advanced) {exit = std::min(exit, value);}
        }
      }
      if (!std::isfinite(exit)) {return 0.;}
      advanced = exit+.002;
    }
    return 0.;
  }

  void Update(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (now >= last_ && now-last_ < .1) {return;}
    last_ = now;
    const auto pose = link_->WorldPose();
    nlohmann::json packet = {{"version", 1}, {"sim_time", now}, {"nozzles", nlohmann::json::array()}};
    unsigned blocked = 0;
    double nearest = 4.5;
    const auto array = [](const ignition::math::Vector3d & p) {
      return std::vector<double>{p.X(), p.Y(), p.Z()};
    };
    for (const auto & local : nozzles_) {
      auto grid = local;
      grid.origin = pose.CoordPositionAdd(local.origin);
      grid.direction = pose.Rot().RotateVector(local.direction);
      grid.u = pose.Rot().RotateVector(local.u);
      grid.v = pose.Rot().RotateVector(local.v);
      for (unsigned y = 0; y < grid.size; ++y) {
        for (unsigned x = 0; x < grid.size; ++x) {
          const double sx = (2.*x/(grid.size-1)-1.)*grid.tangent;
          const double sy = (2.*y/(grid.size-1)-1.)*grid.tangent;
          const auto direction = (grid.direction+grid.u*sx+grid.v*sy).Normalized();
          const double distance = Distance(grid.origin, direction);
          grid.ranges.push_back(distance);
          blocked += distance < 4.49;
          nearest = std::min(nearest, distance);
        }
      }
      packet["nozzles"].push_back({{"origin", array(grid.origin)}, {"direction", array(grid.direction)},
        {"u", array(grid.u)}, {"v", array(grid.v)}, {"size", grid.size},
        {"tangent", grid.tangent}, {"ranges", grid.ranges}});
    }
    gazebo::msgs::GzString message;
    message.set_data(packet.dump());
    publisher_->Publish(message);
    if (now < logged_ || now-logged_ >= 5.) {
      gzmsg << "Spray physical occlusion: blocked_rays=" << blocked
            << "/" << nozzles_.size()*81 << " nearest_m=" << nearest << "\n";
      logged_ = now;
    }
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::RayShapePtr ray_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::PublisherPtr publisher_;
  gazebo::event::ConnectionPtr update_;
  std::vector<SprayDepthGrid> nozzles_;
  double last_{-1.}, logged_{-5.};
};
GZ_REGISTER_MODEL_PLUGIN(WorkSprayObstacles)
}  // namespace agribot_hil
