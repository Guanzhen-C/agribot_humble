#include <cmath>
#include <memory>
#include <gtest/gtest.h>
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
#include "position_only_rpp_controller.hpp"

using agribot_vehicle_description::detail::PositionOnlyGoalCheckerView;

class NativeRppRotationProbe
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  NativeRppRotationProbe()
  {
    use_rotate_to_heading_ = true;
    rotate_to_heading_min_angle_ = 0.785;
  }
  void useTolerance(nav2_core::GoalChecker & checker)
  {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
    ASSERT_TRUE(checker.getTolerances(pose, velocity));
    goal_dist_tol_ = pose.position.x;
  }
  bool finalRotation(const geometry_msgs::msg::PoseStamped & carrot)
  {
    return shouldRotateToGoalHeading(carrot);
  }
  bool pathRotation(const geometry_msgs::msg::PoseStamped & carrot)
  {
    double angle = 0;
    return shouldRotateToPath(carrot, angle);
  }
};

class PositionOnlyRpp : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("position_only_rpp_test");
    node->declare_parameter("goal.xy_goal_tolerance", 1.782);
    node->declare_parameter("goal.stateful", true);
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
    checker = loader->createSharedInstance("nav2_controller::PositionGoalChecker");
    checker->initialize(node, "goal", nullptr);
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> loader;
  nav2_core::GoalChecker::Ptr checker;
};

TEST_F(PositionOnlyRpp, ReproducesNativeLargeToleranceRotationAtRouteStart)
{
  NativeRppRotationProbe native;
  native.useTolerance(*checker);
  geometry_msgs::msg::PoseStamped carrot;
  carrot.pose.position.x = 1.730;
  carrot.pose.position.y = 0.071;
  EXPECT_TRUE(native.finalRotation(carrot));
  PositionOnlyGoalCheckerView view(*checker);
  native.useTolerance(view);
  EXPECT_FALSE(native.finalRotation(carrot));
}

TEST_F(PositionOnlyRpp, InitialPathHeadingAlignmentIsPreserved)
{
  PositionOnlyGoalCheckerView view(*checker);
  NativeRppRotationProbe native;
  native.useTolerance(view);
  geometry_msgs::msg::PoseStamped carrot;
  carrot.pose.position.y = 2.0;
  EXPECT_TRUE(native.pathRotation(carrot));
  EXPECT_FALSE(native.finalRotation(carrot));
}

TEST_F(PositionOnlyRpp, RealHalfWheelbaseAcceptanceAndReversedHeadingAreUnchanged)
{
  PositionOnlyGoalCheckerView view(*checker);
  geometry_msgs::msg::Pose tolerance, goal, current;
  geometry_msgs::msg::Twist tolerance_velocity, velocity;
  velocity.linear.x = 0.4;
  goal.orientation.w = 1;
  current.orientation.z = 1;
  ASSERT_TRUE(view.getTolerances(tolerance, tolerance_velocity));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 0.0);
  ASSERT_TRUE(checker->getTolerances(tolerance, tolerance_velocity));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 1.782);
  current.position.x = 1.783;
  EXPECT_FALSE(view.isGoalReached(current, goal, velocity));
  current.position.x = 1.781;
  EXPECT_TRUE(view.isGoalReached(current, goal, velocity));
}

TEST_F(PositionOnlyRpp, ResetIsDelegatedToTheOriginalChecker)
{
  PositionOnlyGoalCheckerView view(*checker);
  geometry_msgs::msg::Pose goal, current;
  geometry_msgs::msg::Twist velocity;
  EXPECT_TRUE(view.isGoalReached(current, goal, velocity));
  view.reset();
  current.position.x = 2.0;
  EXPECT_FALSE(view.isGoalReached(current, goal, velocity));
}

TEST_F(PositionOnlyRpp, InternalViewCannotReinitializeTheOwnedChecker)
{
  PositionOnlyGoalCheckerView view(*checker);
  EXPECT_THROW(view.initialize(node, "other", nullptr), std::logic_error);
}

TEST_F(PositionOnlyRpp, ControllerIsLoadableAndRequiresARealGoalChecker)
{
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
  auto controller = controllers.createSharedInstance(
    "agribot_vehicle_description::PositionOnlyRegulatedPurePursuitController");
  ASSERT_NE(controller, nullptr);
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  EXPECT_THROW(controller->computeVelocityCommands(pose, speed, nullptr), nav2_core::PlannerException);
}
