#include "data_integrate/tasks/blue_ball_search_task.h"

#include <ros/ros.h>

BlueBallSearchTask::BlueBallSearchTask() : line_driver_(10, -10, 0.01), turn_driver_(1, 0.02, 0.05)
{
}

Task::TaskPtr BlueBallSearchTask::tick(Blackboard &blackboard)
{
  // 계획 중인 상태 전환
  // -> BlueBallCaptureTask   : ★ 파란 공을 발견했을 경우
  // -> GoalPostSearchTask    : 파란 공을 어쩌다 보니(?) 포획했을 경우 [가능성 낮음]
  // -> BlueBallDeliverTask   :(없음)

  if (!has_started_)
  {
    double distance = 5;
    ROS_INFO("Set line driver move distance to %f (m)", distance);
    line_driver_.setDistance(distance);
    has_started_ = true;
  }

  ROS_INFO("Line driver is driving...");
  line_driver_.updateController(blackboard.wheel_controller_, blackboard.getTimeUntilNextTick());

  return nullptr;
}
