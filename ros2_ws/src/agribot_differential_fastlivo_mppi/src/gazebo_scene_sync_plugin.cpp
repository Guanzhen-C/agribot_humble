#include <functional>
#include <queue>
#include <string>
#include <utility>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/RenderingIface.hh>

namespace gazebo
{
class AgribotDifferentialGazeboSceneSync : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    world_ = std::move(world);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(
        &AgribotDifferentialGazeboSceneSync::OnUpdate,
        this,
        std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!world_ || info.simTime < next_update_time_) {
      return;
    }
    next_update_time_ = info.simTime + common::Time(0, 5000000);

    msgs::PosesStamped poses;
    msgs::Set(poses.mutable_time(), info.simTime);

    std::queue<physics::ModelPtr> models;
    for (const auto & model : world_->Models()) {
      models.push(model);
    }

    while (!models.empty()) {
      const auto model = models.front();
      models.pop();

      auto * model_pose = poses.add_pose();
      model_pose->set_name(model->GetScopedName());
      model_pose->set_id(model->GetId());
      msgs::Set(model_pose, model->RelativePose());

      for (const auto & link : model->GetLinks()) {
        auto * link_pose = poses.add_pose();
        link_pose->set_name(link->GetScopedName());
        link_pose->set_id(link->GetId());
        msgs::Set(link_pose, link->RelativePose());
      }

      for (const auto & nested_model : model->NestedModels()) {
        models.push(nested_model);
      }
    }

    rendering::update_scene_poses(world_->Name(), poses);
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  common::Time next_update_time_;
};

GZ_REGISTER_WORLD_PLUGIN(AgribotDifferentialGazeboSceneSync)
}  // namespace gazebo
