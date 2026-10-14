#include <gtest/gtest.h>
#include "../src/ackermann_trajectory_generator.cpp"

class TestGenerator : public agribot_vehicle_description::AckermannTrajectoryGenerator
{
public:
  using AckermannTrajectoryGenerator::computeNewVelocity;
};

TEST(AckermannGenerator, SamplesAndAccelerationsRespectCurvature)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2_util::LifecycleNode>("generator_test");
  node->declare_parameter("FollowPath.min_turning_radius", 1.3032463);
  node->declare_parameter("FollowPath.min_vel_x", 0.0);
  node->declare_parameter("FollowPath.max_vel_x", 0.3);
  node->declare_parameter("FollowPath.max_vel_theta", 0.23);
  node->declare_parameter("FollowPath.max_speed_xy", 0.3);
  node->declare_parameter("FollowPath.acc_lim_x", 0.8);
  node->declare_parameter("FollowPath.decel_lim_x", -0.8);
  node->declare_parameter("FollowPath.acc_lim_theta", 0.35);
  node->declare_parameter("FollowPath.decel_lim_theta", -0.35);
  TestGenerator generator;
  generator.initialize(node, "FollowPath");
  nav_2d_msgs::msg::Twist2D current;
  generator.startNewIteration(current);
  std::size_t samples = 0;
  bool left = false, right = false, straight = false;
  while (generator.hasMoreTwists()) {
    const auto command = generator.nextTwist();
    EXPECT_GE(command.x, 0.02);
    EXPECT_EQ(command.y, 0.0);
    EXPECT_LE(std::abs(command.theta), command.x / 1.3032463 + 1e-8);
    left |= command.theta > 0.0;
    right |= command.theta < 0.0;
    straight |= command.theta == 0.0;
    current = nav_2d_msgs::msg::Twist2D();
    for (int i = 0; i < 30; ++i) {
      current = generator.computeNewVelocity(command, current, 0.02);
      EXPECT_EQ(current.y, 0.0);
      EXPECT_LE(std::abs(current.theta), std::abs(current.x) / 1.3032463 + 1e-8);
    }
    ++samples;
  }
  EXPECT_GT(samples, 20u);
  EXPECT_TRUE(left && right && straight);
  rclcpp::shutdown();
}
