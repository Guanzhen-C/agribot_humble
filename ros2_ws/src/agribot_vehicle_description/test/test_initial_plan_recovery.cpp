#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

struct PlanState
{
  int plans = 0;
  int clears = 0;
  bool planner_fails = false;
  bool follower_fails = false;
  bool finished = false;
};

class MockNavigationAction : public BT::ActionNodeBase
{
public:
  MockNavigationAction(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<PlanState> state, bool planner)
  : BT::ActionNodeBase(name, config), state_(state), planner_(planner) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("planner_id"), BT::InputPort<std::string>("controller_id"),
      BT::InputPort<std::string>("goal_checker_id")};
  }

  BT::NodeStatus tick() override
  {
    if (planner_) {
      ++state_->plans;
      setOutput("path", nav_msgs::msg::Path());
      return state_->planner_fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    if (state_->follower_fails) {return BT::NodeStatus::FAILURE;}
    return state_->finished ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void halt() override {setStatus(BT::NodeStatus::IDLE);}

private:
  std::shared_ptr<PlanState> state_;
  bool planner_;
};

class InitialPlanRecovery : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    state = std::make_shared<PlanState>();
    auto lib = std::filesystem::path(ament_index_cpp::get_package_prefix("nav2_behavior_tree")) / "lib";
    for (const auto & plugin : {"pipeline_sequence", "goal_updated_controller", "recovery_node", "round_robin_node"}) {
      factory.registerFromPlugin((lib / (std::string("libnav2_") + plugin + "_bt_node.so")).string());
    }
    for (const auto & name : {"ComputePathToPose", "ComputePathThroughPoses", "FollowPath"}) {
      const bool planner = std::string(name) != "FollowPath";
      factory.registerBuilder<MockNavigationAction>(name,
        [this, planner](const std::string & n, const BT::NodeConfiguration & c) {
          return std::make_unique<MockNavigationAction>(n, c, state, planner);
        });
    }
    factory.registerSimpleAction("ClearEntireCostmap", [this](BT::TreeNode &) {
        ++state->clears;
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::string>("service_name")});
    factory.registerSimpleAction("Wait", [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
      {BT::InputPort<double>("wait_duration")});
    factory.registerSimpleCondition("GoalUpdated", [](BT::TreeNode &) {return BT::NodeStatus::FAILURE;});
    factory.registerSimpleAction("RemovePassedGoals", [](BT::TreeNode & n) {
        n.setOutput("output_goals", n.getInput<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals").value());
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals"), BT::InputPort<double>("radius")});
    board = BT::Blackboard::create();
    board->set("goal", geometry_msgs::msg::PoseStamped());
    board->set("goals", std::vector<geometry_msgs::msg::PoseStamped>(3));
    board->set("number_recoveries", 0);
    tree = factory.createTreeFromFile(GetParam(), board);
  }

  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
  BT::Tree tree;
  std::shared_ptr<PlanState> state;
};

TEST_P(InitialPlanRecovery, ValidTrackingDoesNotPeriodicallyRequestAnotherPlan)
{
  for (int i = 0; i < 1000; ++i) {EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(state->plans, 1);
  state->finished = true;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(state->plans, 1);
}

TEST_P(InitialPlanRecovery, ControllerFailureStillRecoversAndReplans)
{
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  state->follower_fails = true;
  for (int i = 0; i < 100 && state->plans < 2; ++i) {tree.tickRoot();}
  EXPECT_GT(state->clears, 0);
  EXPECT_GT(state->plans, 1);
  state->follower_fails = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
}

TEST_P(InitialPlanRecovery, InitialPlanningFailureStillClearsAndRetries)
{
  state->planner_fails = true;
  tree.tickRoot();
  EXPECT_GT(state->plans, 1);
  EXPECT_GT(state->clears, 0);
  state->planner_fails = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
}

TEST_P(InitialPlanRecovery, NewRequestCanReplaceTheInitialRoute)
{
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 9.0;
  board->set("goal", goal);
  auto goals = board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
  goals.back() = goal;
  board->set("goals", goals);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(state->plans, 2);
  for (int i = 0; i < 100; ++i) {tree.tickRoot();}
  EXPECT_EQ(state->plans, 2);
}

INSTANTIATE_TEST_SUITE_P(SingleAndThroughPoses, InitialPlanRecovery, ::testing::Values(
    AGRIBOT_TEST_BT_DIR "/navigate_initial_plan_with_recovery.xml",
    AGRIBOT_TEST_BT_DIR "/navigate_through_poses_initial_plan_with_recovery.xml"));
