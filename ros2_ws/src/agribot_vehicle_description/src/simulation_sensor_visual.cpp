#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>

namespace agribot_vehicle_description
{
class SimulationSensorVisual : public gazebo::VisualPlugin
{
public:
  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr) override
  {
    // Show mounted CAD in the GUI without the camera seeing its own housing.
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  }
};
GZ_REGISTER_VISUAL_PLUGIN(SimulationSensorVisual)
}  // namespace agribot_vehicle_description
