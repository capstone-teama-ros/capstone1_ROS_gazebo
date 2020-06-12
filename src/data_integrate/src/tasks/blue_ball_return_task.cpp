#include "data_integrate/tasks/blue_ball_return_task.h"

#include <ros/ros.h>

Task::TaskPtr BlueBallReturnTask::updateTaskOrMakeNextTask(double time_passed_after_last,
                                                           const VisibleFeatureManager &visible_features,
                                                           const PastFeatureManager &past_features)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");

  // 계획 중인 상태 전환
  // -> BlueBallSearchTask    : ★ 파란 공을 골대에 넣었을 경우 or 파란 공을 떨어뜨렸을 경우
  // -> BlueBallCaptureTask   : (없음)
  // -> GoalPostSearchTask    : 골대가 갑자기 사라졌을 경우 [가능성 낮음]
}

void BlueBallReturnTask::updateWheelController(double time_until_next, SimpleWheelController &wheel_controller)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");
}
