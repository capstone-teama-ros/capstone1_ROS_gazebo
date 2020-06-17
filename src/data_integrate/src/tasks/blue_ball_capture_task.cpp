#include "data_integrate/tasks/blue_ball_capture_task.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include "data_integrate/tasks/turn_to_nearest_blue_ball.h"

BlueBallCaptureTask::BlueBallCaptureTask()
{
  const double MAX_TURN_SPEED = angles::from_degrees(30);
  const double THRESHOLD = angles::from_degrees(0.5);

  subtasks_.push_back(TaskPtr(new TurnToNearestBlueBall(MAX_TURN_SPEED, THRESHOLD)));

  current_subtask_ = subtasks_.begin();
}

TaskResult BlueBallCaptureTask::doTick(Blackboard &blackboard)
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

void BlueBallCaptureTask::doHalt(Blackboard &blackboard)
{
  for (auto &subtask : subtasks_)
  {
    subtask->halt(blackboard);
  }
  current_subtask_ = subtasks_.begin();
}
