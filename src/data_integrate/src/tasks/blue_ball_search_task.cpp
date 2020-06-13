#include "data_integrate/tasks/blue_ball_search_task.h"

#include <ros/ros.h>

TaskResult BlueBallSearchTask::tick(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");

  // 계획 중인 상태 전환
  // -> BlueBallCaptureTask   : ★ 파란 공을 발견했을 경우
  // -> GoalPostSearchTask    : 파란 공을 어쩌다 보니(?) 포획했을 경우 [가능성 낮음]
  // -> BlueBallDeliverTask   :(없음)
}

void BlueBallSearchTask::halt(Blackboard &blackboard)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");
}
