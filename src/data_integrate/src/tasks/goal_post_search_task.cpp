#include "data_integrate/tasks/goal_post_search_task.h"

#include <ros/ros.h>

TaskResult GoalPostSearchTask::doTick(Blackboard &blackboard)
{
  // Finish if the green ball is found
  if (!blackboard.visible_features_.getBalls(BallColor::Green).empty())
  {
    ROS_INFO("%s: Found green ball", name());
    halt(blackboard);
    return TaskResult::Success;
  }

  // Abort if we lose the blue ball
  if (!blackboard.visible_features_.isBlueBallCaptured())
  {
    ROS_WARN("%s: We dropped blue ball", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  // Keep rotating
  auto result = turn_task_.tick(blackboard);
  if (result == TaskResult::Success || result == TaskResult::Failure)
  {
    // If we reach here, it means we turned 360 degrees but couldn't find the green ball.
    // Or the turning was interrupted for some reason.
    ROS_ERROR("%s: Could not find green ball", name());
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

void GoalPostSearchTask::doHalt(Blackboard &blackboard)
{
  turn_task_.halt(blackboard);
}
