#include <algorithm>
#include <cmath>
#include <mutex>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>

namespace agribot_hitch {
class LinkageVisual : public gazebo::VisualPlugin {
public:
  ~LinkageVisual() override
  {
    update_.reset(); pose_subscription_.reset();
    for (auto &part : parts_) {if (part) {part->Fini();}}
  }
  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    visual_ = visual;
    host_ = sdf->Get<ignition::math::Vector3d>("host");
    eye_ = sdf->Get<ignition::math::Vector3d>("tool_eye");
    radius_ = sdf->Get<double>("radius");
    tool_pose_ = sdf->Get<ignition::math::Pose3d>("initial_tool_pose");
    const auto parent_name = visual_->GetParent()->Name();
    tool_name_ = parent_name.substr(0, parent_name.rfind("::"))+"::hitch_tool_frame";
    for (unsigned i = 0; i < 6; ++i) {
      // Use the link's metre-valued frame, not the tiny anchor sphere's scale.
      parts_[i].reset(new gazebo::rendering::Visual(visual_->Name()+"_part_"+std::to_string(i), visual_->GetParent()));
      parts_[i]->Load();
      const std::string shape = i == 3 || i == 4 ? "unit_sphere" : "unit_cylinder";
      parts_[i]->InsertMesh(shape); parts_[i]->AttachMesh(shape);
      parts_[i]->SetMaterial("Gazebo/Grey");
      parts_[i]->SetAmbient(i == 1 ? ignition::math::Color(.24, .25, .25, 1) :
                                     ignition::math::Color(.10, .12, .11, 1));
      parts_[i]->SetDiffuse(i == 1 ? ignition::math::Color(.66, .69, .68, 1) :
                                     ignition::math::Color(.27, .31, .29, 1));
      parts_[i]->SetCastShadows(true);
    }
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(sdf->Get<std::string>("world_name", "hitch_farmland").first);
    pose_subscription_ = transport_->Subscribe("~/hil_hitch/lift_pose", &LinkageVisual::Pose, this);
    update_ = gazebo::event::Events::ConnectPreRender([this]() {Render();});
    gzmsg << "Telescopic hitch linkage attached: " << visual_->Name() << "\n";
  }
private:
  void Pose(ConstPosePtr &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tool_pose_ = gazebo::msgs::ConvertIgn(*message);
  }
  void Cylinder(unsigned index, const ignition::math::Vector3d &a,
                const ignition::math::Vector3d &b, double radius)
  {
    const auto delta = b-a;
    ignition::math::Quaterniond rotation;
    rotation.From2Axes(ignition::math::Vector3d::UnitZ, delta.Normalized());
    parts_[index]->SetPose(ignition::math::Pose3d((a+b)*.5, rotation));
    parts_[index]->SetScale({2*radius, 2*radius, delta.Length()});
  }
  void Render()
  {
    ignition::math::Pose3d pose;
    {std::lock_guard<std::mutex> lock(mutex_); pose = tool_pose_;}
    // Use the same render frames as the CAD clevis, including late GUI joins.
    const auto tool = visual_->GetScene()->GetVisual(tool_name_);
    if (tool) {pose = visual_->GetParent()->WorldPose().Inverse()*tool->WorldPose();}
    const auto end = pose.CoordPositionAdd(eye_);
    const auto delta = end-host_;
    const double length = delta.Length();
    if (!std::isfinite(length) || length < .05 || length > 3.) {return;}
    const auto direction = delta/length;
    const auto mouth = host_+direction*(length*.60);
    // The inner rod overlaps the sleeve, so extension never opens a gap.
    Cylinder(0, host_, mouth, radius_);
    Cylinder(1, host_+direction*(length*.45), end, radius_*.62);
    Cylinder(2, mouth-direction*.015, mouth+direction*.015, radius_*1.12);
    parts_[3]->SetPosition(host_); parts_[4]->SetPosition(end);
    parts_[3]->SetScale({radius_*2.4, radius_*2.4, radius_*2.4});
    parts_[4]->SetScale({radius_*2.4, radius_*2.4, radius_*2.4});
    Cylinder(5, end+ignition::math::Vector3d(0, .06, 0),
                end-ignition::math::Vector3d(0, .06, 0), radius_*.42);
  }
  gazebo::rendering::VisualPtr visual_, parts_[6];
  std::string tool_name_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::SubscriberPtr pose_subscription_;
  gazebo::event::ConnectionPtr update_;
  std::mutex mutex_;
  ignition::math::Vector3d host_, eye_;
  ignition::math::Pose3d tool_pose_;
  double radius_{.025};
};
GZ_REGISTER_VISUAL_PLUGIN(LinkageVisual)
}
