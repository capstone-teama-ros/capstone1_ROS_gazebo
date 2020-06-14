#include "data_integrate/tasks/periodic_search_move_once.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include "data_integrate/tasks/move_distance.h"
#include "data_integrate/tasks/turn_angle.h"

PeriodicSearchMoveOnce::PeriodicSearchMoveOnce()
{
  subtasks_.push_back(TaskPtr(new TurnAngle(angles::from_degrees(90))));
  subtasks_.push_back(TaskPtr(new TurnAngle(angles::from_degrees(-90))));
  subtasks_.push_back(TaskPtr(new MoveDistance(1)));

  current_subtask_ = subtasks_.begin();
}

void PeriodicSearchMoveOnce::doHalt(Blackboard &blackboard)
{
  for (auto &subtask : subtasks_)
  {
    subtask->halt(blackboard);
  }
  current_subtask_ = subtasks_.begin();
}

TaskResult PeriodicSearchMoveOnce::doTick(Blackboard &blackboard)
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

  halt(blackboard);
  return TaskResult::Success;
}
