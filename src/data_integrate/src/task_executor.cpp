#include "data_integrate/task_executor.h"

#include <ros/ros.h>

void TaskExecutor::runTaskInLoop(double time_passed_after_last, double time_until_next)
{
  ROS_ASSERT_MSG(task_, "Missing task");

  // 현재 작업을 업데이트하고, 필요하면 새로운 작업으로 전환합니다.
  auto nextTask = task_->updateTaskOrMakeNextTask(time_passed_after_last);
  if (nextTask)
  {
    task_ = nextTask;
  }

  // 업데이트한 작업의 결정에 따라 바퀴 컨트롤러의 상태를 업데이트합니다.
  task_->updateWheelController(wheel_controller);
  // 바퀴 컨트롤러를 통해 메시지를 전파합니다.
  wheel_controller.publish();
}

void TaskExecutor::overrideTask(Task::TaskPtr task)
{
  task_ = task;
}
