#include <gtest/gtest.h>
#include "navigation_task_state.hpp"

action_msgs::msg::GoalStatusArray status(int seconds, int value, uint8_t identity = 0)
{
  action_msgs::msg::GoalStatusArray message;
  action_msgs::msg::GoalStatus goal;
  goal.goal_info.stamp.sec = seconds;
  goal.goal_info.goal_id.uuid[0] = identity;
  goal.status = value;
  message.status_list.push_back(goal);
  return message;
}

TEST(NavigationTaskState, ClearsAtNavigatorSuccessAndOpensForNewTask)
{
  scout_viz::NavigationTaskState state;
  EXPECT_FALSE(state.update("navigate_to_pose", status(10, 2)));
  EXPECT_FALSE(state.hidden());
  EXPECT_TRUE(state.update("navigate_to_pose", status(10, 4)));
  EXPECT_TRUE(state.hidden());
  EXPECT_FALSE(state.update("navigate_to_pose", status(10, 4)));
  EXPECT_FALSE(state.update("navigate_to_pose", status(20, 2)));
  EXPECT_FALSE(state.hidden());
}

TEST(NavigationTaskState, CancelAndAbortAlsoClear)
{
  for (int value : {5, 6}) {
    scout_viz::NavigationTaskState state;
    state.update("navigate_through_poses", status(10, 2));
    EXPECT_TRUE(state.update("navigate_through_poses", status(10, value)));
    EXPECT_TRUE(state.hidden());
  }
}

TEST(NavigationTaskState, IntermediateControllerCompletionDoesNotClearParentTask)
{
  scout_viz::NavigationTaskState state;
  state.update("navigate_through_poses", status(10, 2));
  state.update("follow_path", status(11, 2));
  EXPECT_FALSE(state.update("follow_path", status(11, 4)));
  EXPECT_FALSE(state.hidden());
  state.update("follow_path", status(12, 2));
  EXPECT_FALSE(state.hidden());
  EXPECT_TRUE(state.update("navigate_through_poses", status(10, 4)));
  EXPECT_TRUE(state.hidden());
}

TEST(NavigationTaskState, WaypointFollowerChildrenDoNotClearTheWholeRoute)
{
  scout_viz::NavigationTaskState state;
  state.update("follow_waypoints", status(10, 2));
  state.update("navigate_to_pose", status(11, 2));
  EXPECT_FALSE(state.update("navigate_to_pose", status(11, 4)));
  state.update("navigate_to_pose", status(12, 2));
  EXPECT_FALSE(state.update("follow_path", status(13, 4)));
  EXPECT_FALSE(state.hidden());
  EXPECT_TRUE(state.update("follow_waypoints", status(10, 4)));
}

TEST(NavigationTaskState, OldTerminalFromAnotherActionCannotClearTheNewRoute)
{
  scout_viz::NavigationTaskState state;
  state.update("navigate_to_pose", status(10, 2));
  state.update("navigate_through_poses", status(20, 2));
  EXPECT_FALSE(state.update("navigate_to_pose", status(10, 4)));
  EXPECT_FALSE(state.hidden());
}

TEST(NavigationTaskState, OldActiveStatusCannotReopenAnEndedGoal)
{
  scout_viz::NavigationTaskState state;
  state.update("navigate_to_pose", status(10, 4));
  state.update("navigate_to_pose", status(10, 2));
  EXPECT_TRUE(state.hidden());
  state.update("navigate_to_pose", status(9, 2));
  EXPECT_TRUE(state.hidden());
}

TEST(NavigationTaskState, NewestGoalInStatusHistoryWins)
{
  scout_viz::NavigationTaskState state;
  auto history = status(20, 2);
  history.status_list.push_back(status(10, 4).status_list.front());
  state.update("navigate_to_pose", history);
  EXPECT_FALSE(state.hidden());
  EXPECT_FALSE(state.update("navigate_to_pose", action_msgs::msg::GoalStatusArray()));
}

TEST(NavigationTaskState, DirectFollowPathIsSupported)
{
  scout_viz::NavigationTaskState state;
  state.update("follow_path", status(10, 2));
  EXPECT_TRUE(state.update("follow_path", status(10, 4)));
  EXPECT_TRUE(state.hidden());
  state.update("follow_path", status(20, 2));
  EXPECT_FALSE(state.hidden());
}

TEST(NavigationTaskState, LateControllerActivityCannotReopenAFinishedNavigator)
{
  scout_viz::NavigationTaskState state;
  state.update("navigate_through_poses", status(10, 2));
  state.update("navigate_through_poses", status(10, 4));
  state.update("follow_path", status(11, 2));
  EXPECT_TRUE(state.hidden());
  state.update("navigate_through_poses", status(20, 2));
  EXPECT_FALSE(state.hidden());
}

TEST(NavigationTaskState, LateParentActiveCanAdoptAnActiveChild)
{
  scout_viz::NavigationTaskState state;
  state.update("follow_path", status(11, 2));
  state.update("navigate_to_pose", status(10, 2));
  EXPECT_FALSE(state.update("follow_path", status(11, 4)));
  EXPECT_FALSE(state.hidden());
  EXPECT_TRUE(state.update("navigate_to_pose", status(10, 4)));
}

TEST(NavigationTaskState, ResetAllowsAResetSimulationClock)
{
  scout_viz::NavigationTaskState state;
  state.update("navigate_to_pose", status(100, 4));
  state = scout_viz::NavigationTaskState();
  state.update("navigate_to_pose", status(1, 2));
  EXPECT_FALSE(state.hidden());
}
