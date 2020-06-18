#include "data_integrate/tasks/ball_harvester.h"

#include "data_integrate/tasks/blue_ball_capture_task.h"
#include "data_integrate/tasks/blue_ball_return_task.h"
#include "data_integrate/tasks/blue_ball_search_task.h"
#include "data_integrate/tasks/goal_post_search_task.h"

BallHarvester::BallHarvester()
{
  subtasks_.push_back(TaskPtr(new BlueBallSearchTask()));
  subtasks_.push_back(TaskPtr(new BlueBallCaptureTask()));
  subtasks_.push_back(TaskPtr(new GoalPostSearchTask()));
  subtasks_.push_back(TaskPtr(new BlueBallReturnTask()));

  current_subtask_ = subtasks_.begin();
}

TaskResult BallHarvester::doTick(Blackboard &blackboard)
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

void BallHarvester::doHalt(Blackboard &blackboard)
{
  for (auto &subtask : subtasks_)
  {
    subtask->halt(blackboard);
  }
  current_subtask_ = subtasks_.begin();
}
