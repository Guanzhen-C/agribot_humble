#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace agribot_vehicle_description
{
class SimulationSensorCollisionFilter : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override
  {
    // A whole-CAD bounding box fills empty space around mounted sensors.
    // Exclude only this model's proxies from ODE rays, not from ground or
    // obstacle contact. World collision geometry remains visible to LiDAR.
    for (const auto & link : model->GetLinks()) {
      for (const auto & collision : link->GetCollisions()) {
        collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
        collision->SetCollideBits(~GZ_SENSOR_COLLIDE);
      }
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(SimulationSensorCollisionFilter)
}  // namespace agribot_vehicle_description
