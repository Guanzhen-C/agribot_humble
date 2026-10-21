#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <gtest/gtest.h>
#include "../src/route_progress_goal_checker.cpp"

using agribot_vehicle_description::RouteProgressGoalChecker;

class RouteGoalTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("route_goal_test");
    sender = std::make_shared<rclcpp::Node>("route_goal_sender");
    publisher = sender->create_publisher<std_msgs::msg::Bool>(
      "/waypoint_goal_ready", rclcpp::QoS(1).reliable().transient_local());
    requested_publisher = sender->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/route_requested_goal_test", rclcpp::QoS(1).reliable().transient_local());
    goal.orientation.w = 1.0;
  }

  void initialize(
    const std::string & wrapped = "nav2_controller::SimpleGoalChecker",
    bool requested_goal = false, double tolerance = 0.25)
  {
    node->declare_parameter("guard.wrapped_plugin", wrapped);
    node->declare_parameter("guard.xy_goal_tolerance", tolerance);
    node->declare_parameter("guard.yaw_goal_tolerance", 0.20);
    if (requested_goal) {
      node->declare_parameter("guard.requested_goal_topic", "/route_requested_goal_test");
      node->declare_parameter("guard.requested_goal_frame", "map");
    }
    checker.initialize(node, "guard", nullptr);
  }

  void primeRequestedGoal()
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = "map";
    message.pose = goal;
    requested_publisher->publish(message);
    publishReady(true);
  }

  void publishRequestedGoal(
    const geometry_msgs::msg::PoseStamped & message, bool expected_at_origin)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      requested_publisher->publish(message);
      rclcpp::spin_some(node->get_node_base_interface());
      if (checker.isGoalReached(goal, goal, stopped) == expected_at_origin) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    FAIL() << "Requested goal update was not delivered";
  }

  void publishReady(bool ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      std_msgs::msg::Bool message;
      message.data = ready;
      publisher->publish(message);
      rclcpp::spin_some(node->get_node_base_interface());
      if (checker.isGoalReached(goal, goal, stopped) == ready) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    FAIL() << "Route progress update was not delivered";
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<rclcpp::Node> sender;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr requested_publisher;
  RouteProgressGoalChecker checker;
  geometry_msgs::msg::Pose goal;
  geometry_msgs::msg::Twist stopped;
};

TEST_F(RouteGoalTest, FirstPassCannotFinishOrLatchStatefulXYSuccess)
{
  initialize();
  EXPECT_FALSE(checker.isGoalReached(goal, goal, stopped));
  publishReady(true);
  checker.reset();
  auto distant = goal;
  distant.position.x = 1.0;
  EXPECT_FALSE(checker.isGoalReached(distant, goal, stopped));
  EXPECT_TRUE(checker.isGoalReached(goal, goal, stopped));
  publishReady(false);
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(goal, goal, stopped));
}

TEST_F(RouteGoalTest, PreservesNav2TolerancesAndYawCheck)
{
  initialize();
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity;
  ASSERT_TRUE(checker.getTolerances(tolerance, velocity));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 0.25);
  publishReady(true);
  auto wrong_yaw = goal;
  wrong_yaw.orientation.z = std::sin(0.5);
  wrong_yaw.orientation.w = std::cos(0.5);
  EXPECT_FALSE(checker.isGoalReached(wrong_yaw, goal, stopped));
}

TEST_F(RouteGoalTest, PreservesStoppedGoalCheckerVelocityRequirement)
{
  initialize("nav2_controller::StoppedGoalChecker");
  publishReady(true);
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  EXPECT_FALSE(checker.isGoalReached(goal, goal, moving));
  EXPECT_TRUE(checker.isGoalReached(goal, goal, stopped));
}

TEST_F(RouteGoalTest, PositionOnlyCheckerStillIgnoresYaw)
{
  initialize("nav2_controller::PositionGoalChecker");
  publishReady(true);
  auto reversed = goal;
  reversed.orientation.w = 0.0;
  reversed.orientation.z = 1.0;
  EXPECT_TRUE(checker.isGoalReached(reversed, goal, stopped));
}

TEST_F(RouteGoalTest, GridEndpointCannotMoveTheOriginalHalfWheelbaseTolerance)
{
  initialize("nav2_controller::PositionGoalChecker", true, 0.20100000500679016);
  primeRequestedGoal();
  geometry_msgs::msg::PoseStamped requested;
  requested.header.frame_id = "map";
  requested.pose = goal;
  requested.pose.position.x = -8.0;
  publishRequestedGoal(requested, false);
  auto endpoint = requested.pose;
  endpoint.position.x = -8.049999762326479;
  endpoint.position.y = -0.049999747425317764;
  auto current = goal;
  current.position.x = -8.20350607368918;
  current.position.y = -0.1544913860938343;
  ASSERT_LT(std::hypot(current.position.x - endpoint.position.x,
                      current.position.y - endpoint.position.y), 0.201);
  ASSERT_GT(std::hypot(current.position.x + 8.0, current.position.y), 0.201);
  EXPECT_FALSE(checker.isGoalReached(current, endpoint, stopped));
  current.position.x = -8.15;
  current.position.y = -0.10;
  current.orientation.w = 0.0;
  current.orientation.z = 1.0;
  EXPECT_TRUE(checker.isGoalReached(current, endpoint, stopped));
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity;
  ASSERT_TRUE(checker.getTolerances(tolerance, velocity));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 0.20100000500679016);
}

TEST_F(RouteGoalTest, NewRequestedGoalResetsStatefulAcceptanceAndSurvivesActionReset)
{
  initialize("nav2_controller::SimpleGoalChecker", true);
  primeRequestedGoal();
  geometry_msgs::msg::PoseStamped requested;
  requested.header.frame_id = "map";
  requested.pose = goal;
  requested.pose.position.x = 1.0;
  publishRequestedGoal(requested, false);
  EXPECT_FALSE(checker.isGoalReached(goal, goal, stopped));
  checker.reset();
  EXPECT_TRUE(checker.isGoalReached(requested.pose, goal, stopped));
}

TEST_F(RouteGoalTest, InvalidRequestedGoalFailsClosedInsteadOfUsingTheGridEndpoint)
{
  initialize("nav2_controller::PositionGoalChecker", true);
  primeRequestedGoal();
  geometry_msgs::msg::PoseStamped requested;
  requested.pose = goal;
  requested.header.frame_id = "odom";
  publishRequestedGoal(requested, false);
  EXPECT_FALSE(checker.isGoalReached(goal, goal, stopped));
  requested.header.frame_id = "map";
  publishRequestedGoal(requested, true);
  requested.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  publishRequestedGoal(requested, false);
  requested.pose = goal;
  publishRequestedGoal(requested, true);
  requested.pose.orientation.w = 0.0;
  publishRequestedGoal(requested, false);
}

TEST_F(RouteGoalTest, RequestedGoalDoesNotBypassTheOrderedRouteGate)
{
  initialize("nav2_controller::PositionGoalChecker", true);
  primeRequestedGoal();
  publishReady(false);
  EXPECT_FALSE(checker.isGoalReached(goal, goal, stopped));
  publishReady(true);
  EXPECT_TRUE(checker.isGoalReached(goal, goal, stopped));
}

TEST_F(RouteGoalTest, RequestedGoalRequiresAnExplicitControllerFrame)
{
  node->declare_parameter("guard.requested_goal_topic", "/route_requested_goal_test");
  EXPECT_THROW(initialize(), std::runtime_error);
}
