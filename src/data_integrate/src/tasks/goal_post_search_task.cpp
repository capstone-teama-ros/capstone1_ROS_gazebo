#include "data_integrate/tasks/goal_post_search_task.h"

#include <ros/ros.h>

Task::TaskPtr GoalPostSearchTask::updateTaskOrMakeNextTask(double time_passed_after_last,
                                                           const VisibleFeatureManager &visible_features)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");

  // 계획 중인 상태 전환
  // -> BlueBallSearchTask    : 파란 공을 떨어뜨렸을 경우
  // -> BlueBallCaptureTask   : (없음)
  // -> BlueBallDeliverTask   : ★ 골대를 찾았을 경우
}

void GoalPostSearchTask::updateWheelController(double time_until_next, SimpleWheelController &wheel_controller)
{
  // TODO 실제 코드를 추가해야 합니다
  ROS_ASSERT_MSG(0, "Not implemented");
}
