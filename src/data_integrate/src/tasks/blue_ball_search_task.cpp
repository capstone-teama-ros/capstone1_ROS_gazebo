#include "data_integrate/tasks/blue_ball_search_task.h"

#include <angles/angles.h>
#include <ros/ros.h>

BlueBallSearchTask::BlueBallSearchTask()
  : line_driver_(10, -10, 0.01), turn_driver_(1, 0.02, 0.05), square_driver_(5, 2.5, angles::from_degrees(30), 5)
{
}

Task::TaskPtr BlueBallSearchTask::tick(Blackboard &blackboard)
{
  // 계획 중인 상태 전환
  // -> BlueBallCaptureTask   : ★ 파란 공을 발견했을 경우
  // -> GoalPostSearchTask    : 파란 공을 어쩌다 보니(?) 포획했을 경우 [가능성 낮음]
  // -> BlueBallDeliverTask   :(없음)

  square_driver_.updateState(blackboard.getTimeSinceLastTick());
  square_driver_.updateController(blackboard.wheel_controller_);

  return nullptr;
}
