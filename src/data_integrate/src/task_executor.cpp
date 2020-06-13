#include "data_integrate/task_executor.h"

#include <ros/ros.h>
#include <utility>

void TaskExecutor::runTaskInLoop(double time_passed_after_last, double time_until_next)
{
  ROS_ASSERT_MSG(task_, "Missing task");

  // 현재 작업을 업데이트하고, 필요하면 새로운 작업으로 전환합니다.
  blackboard_.setTimeSinceLastTick(time_passed_after_last);
  blackboard_.setTimeUntilNextTick(time_until_next);
  auto nextTask = task_->tick(blackboard_);
  if (nextTask)
  {
    task_ = std::move(nextTask);
  }

  // TODO: 여러 task/node가 동시에 바퀴 컨트롤러를 조작하지 못하게 방지하는 장치가 필요하다.

  // 바퀴 컨트롤러를 통해 메시지를 전파합니다.
  blackboard_.wheel_controller_.publish();
}

void TaskExecutor::overrideTask(Task::TaskPtr task)
{
  task_ = std::move(task);
}
