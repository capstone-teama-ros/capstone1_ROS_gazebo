#include "data_integrate/tasks/periodic_search_move_repeat.h"

#include <ros/ros.h>
#include "data_integrate/tasks/periodic_search_move_once.h"

PeriodicSearchMoveRepeat::PeriodicSearchMoveRepeat() : subtask_(new PeriodicSearchMoveOnce())
{
}

void PeriodicSearchMoveRepeat::doHalt(Blackboard &blackboard)
{
  subtask_->halt(blackboard);
  subtask_repeat_count_ = 0;
}

TaskResult PeriodicSearchMoveRepeat::doTick(Blackboard &blackboard)
{
  while (subtask_repeat_count_ < subtask_repeat_max_)
  {
    auto result = subtask_->tick(blackboard);

    if (result == TaskResult::Success)
    {
      ++subtask_repeat_count_;
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

  halt(blackboard);
  return TaskResult::Success;
}
