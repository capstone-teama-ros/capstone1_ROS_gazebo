#include "data_integrate/task_executor.h"

#include <ros/ros.h>
#include <utility>

void TaskExecutor::runTaskInLoop(double time_passed_after_last, double time_until_next)
{
  ROS_ASSERT_MSG(task_, "Missing task");

  // 현재 작업을 업데이트합니다.
  blackboard_.setTimeSinceLastTick(time_passed_after_last);
  blackboard_.setTimeUntilNextTick(time_until_next);
  auto result = task_->tick(blackboard_);

  if (result == TaskResult::Success)
  {
    ROS_DEBUG("Root task has succeeded");
  }
  else if (result == TaskResult::Failure)
  {
    ROS_WARN("Root task has failed");
  }
  else if (result == TaskResult::Running)
  {
    ROS_DEBUG("Root task is running");
  }
  else
    ROS_ASSERT_MSG(0, "Invalid task result: %u", static_cast<unsigned int>(result));

  // TODO: 여러 task/node가 동시에 바퀴 컨트롤러를 조작하지 못하게 방지하는 장치가 필요하다.

  // 바퀴 컨트롤러를 통해 메시지를 전파합니다.
  blackboard_.wheel_controller_.publish();
}

void TaskExecutor::overrideTask(Task::TaskPtr task)
{
  task_ = std::move(task);
}
