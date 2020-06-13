#include "data_integrate/tasks/blue_ball_capture_task.h"

#include <ros/ros.h>

TaskResult BlueBallCaptureTask::doTick(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");

  // 계획 중인 상태 전환
  // -> BlueBallSearchTask    : 파란 공을 떨어뜨렸을 경우
  // -> GoalPostSearchTask    : ★ 파란 공을 포획했을 경우
  // -> BlueBallDeliverTask   : (없음)
}

void BlueBallCaptureTask::doHalt(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");
}
