#include "data_integrate/tasks/blue_ball_return_task.h"

#include <ros/ros.h>
#include "data_integrate/tasks/kick_ball_into_goal.h"
#include "data_integrate/tasks/move_ball_to_goal_area.h"

BlueBallReturnTask::BlueBallReturnTask()
{
  subtasks_.push_back(TaskPtr(new MoveBallToGoalArea()));
  subtasks_.push_back(TaskPtr(new KickBallIntoGoal()));

  current_subtask_ = subtasks_.begin();
}

TaskResult BlueBallReturnTask::doTick(Blackboard &blackboard)
{
  // 모두 성공할 때까지 순차적으로 실행
  while (current_subtask_ != subtasks_.end())
  {
    auto result = (*current_subtask_)->tick(blackboard);

    if (result == TaskResult::Success)
    {
      ++current_subtask_;
    }
    else if (result == TaskResult::Failure)
    {
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

  ROS_INFO("%s succeeded!", name());
  halt(blackboard);
  return TaskResult::Success;
}

void BlueBallReturnTask::doHalt(Blackboard &blackboard)
{
  for (auto &subtask : subtasks_)
  {
    subtask->halt(blackboard);
  }
  current_subtask_ = subtasks_.begin();
}
