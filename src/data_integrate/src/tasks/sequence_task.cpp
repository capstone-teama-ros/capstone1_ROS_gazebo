#include "data_integrate/tasks/sequence_task.h"

#include <ros/ros.h>
#include <cstdarg>
#include <memory>

SequenceTask::SequenceTask(int count, ...)
{
  va_list args;
  va_start(args, count);

  for (int i = 0; i < count; ++i)
  {
    subtasks_.emplace_back(std::move(va_arg(args, TaskPtr)));
  }
  va_end(args);

  current_subtask_ = subtasks_.begin();
}

TaskResult SequenceTask::doTick(Blackboard &blackboard)
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

void SequenceTask::doHalt(Blackboard &blackboard)
{
  for (auto &subtask : subtasks_)
  {
    subtask->halt(blackboard);
  }
  current_subtask_ = subtasks_.begin();
}
