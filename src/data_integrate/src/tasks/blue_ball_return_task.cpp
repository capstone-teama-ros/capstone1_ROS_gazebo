#include "data_integrate/tasks/blue_ball_return_task.h"

#include <ros/ros.h>

TaskResult BlueBallReturnTask::tick(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");

  // 계획 중인 상태 전환
  // -> BlueBallSearchTask    : ★ 파란 공을 골대에 넣었을 경우 or 파란 공을 떨어뜨렸을 경우
  // -> BlueBallCaptureTask   : (없음)
  // -> GoalPostSearchTask    : 골대가 갑자기 사라졌을 경우 [가능성 낮음]
}

void BlueBallReturnTask::halt(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");
}
