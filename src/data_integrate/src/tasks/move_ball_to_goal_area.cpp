#include "data_integrate/tasks/move_ball_to_goal_area.h"

#include <ros/ros.h>

TaskResult MoveBallToGoalArea::doTick(Blackboard &blackboard)
{
  // If the robot drops the ball it was holding, fail immediately
  if (!blackboard.visible_features_.isBlueBallCaptured())
  {
    ROS_WARN("%s: Dropped the ball", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  auto result = avoidance_task_.tick(blackboard);
  if (result == TaskResult::Success)
  {
    ROS_INFO("%s: Arrived at goal area", name());
    halt(blackboard);
    return TaskResult::Success;
  }
  else if (result == TaskResult::Failure)
  {
    ROS_INFO("%s: Failed to approach goal area", name());
    halt(blackboard);
    return TaskResult::Failure;
  }
  else if (result == TaskResult::Running)
  {
    return TaskResult::Running;
  }
  else
    ROS_INVALID_TASK_RESULT(result);
}

void MoveBallToGoalArea::doHalt(Blackboard &blackboard)
{
  avoidance_task_.halt(blackboard);
}
