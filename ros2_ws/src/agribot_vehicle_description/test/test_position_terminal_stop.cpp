#include <cmath>
#include <memory>
#include <gtest/gtest.h>
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"

class PositionTerminalStop : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("terminal_position_test");
    node->declare_parameter("terminal.xy_goal_tolerance", 0.25);
    node->declare_parameter("terminal.stateful", true);
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
    checker = loader->createSharedInstance("nav2_controller::PositionGoalChecker");
    checker->initialize(node, "terminal", nullptr);
    goal.orientation.w = 1.0;
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> loader;
  nav2_core::GoalChecker::Ptr checker;
  geometry_msgs::msg::Pose goal;
  geometry_msgs::msg::Twist moving;
};

TEST_F(PositionTerminalStop, ReversedHeadingDoesNotRequireAnotherLap)
{
  auto current = goal;
  current.position.x = 0.24;
  current.orientation.w = 0.0;
  current.orientation.z = 1.0;
  moving.linear.x = 0.5;
  EXPECT_TRUE(checker->isGoalReached(current, goal, moving));
}

TEST_F(PositionTerminalStop, OutsidePositionToleranceStillCannotFinish)
{
  auto current = goal;
  current.position.x = 0.2;
  current.position.y = 0.2;
  EXPECT_FALSE(checker->isGoalReached(current, goal, moving));
}

TEST_F(PositionTerminalStop, ResetDoesNotReusePreviousMissionSuccess)
{
  EXPECT_TRUE(checker->isGoalReached(goal, goal, moving));
  checker->reset();
  auto current = goal;
  current.position.x = 1.0;
  EXPECT_FALSE(checker->isGoalReached(current, goal, moving));
}

TEST_F(PositionTerminalStop, HalfWheelbaseToleranceOnlyChangesPositionAcceptance)
{
  const double tolerance = 2.8137142368632873 * 0.5;
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("terminal.xy_goal_tolerance", tolerance)).successful);
  auto current = goal;
  current.orientation.w = 0.0;
  current.orientation.z = 1.0;
  current.position.x = tolerance - 0.01;
  EXPECT_TRUE(checker->isGoalReached(current, goal, moving));
  checker->reset();
  current.position.x = tolerance + 0.01;
  EXPECT_FALSE(checker->isGoalReached(current, goal, moving));
}
