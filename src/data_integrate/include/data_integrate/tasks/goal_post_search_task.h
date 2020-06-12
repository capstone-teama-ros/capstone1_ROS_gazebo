#ifndef DATA_INTEGRATE_TASKS_GOAL_POST_SEARCH_TASK_H
#define DATA_INTEGRATE_TASKS_GOAL_POST_SEARCH_TASK_H

#include "./task.h"

/**
 * 파란 공을 포획한 이후, 공을 넣을 골대의 위치를 탐색하는 작업입니다.
 */
class GoalPostSearchTask : public Task
{
public:
  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param time_passed_after_last  직전의 상태 업데이트를 호출한 이후, 실제로 지난 시간 (seconds)
   * @param visible_features        가장 최근에 관측된 지형지물 정보
   * @param past_features           기억하고 있는 지형지물 정보
   * @returns 현재 작업을 계속 진행해야 할 경우 @c nullptr 를 리턴합니다.
   * 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업의 객체를 리턴합니다.
   */
  TaskPtr updateTaskOrMakeNextTask(double time_passed_after_last, const VisibleFeatureManager &visible_features,
                                   const PastFeatureManager &past_features) override;

  /**
   * 현재 상태에 알맞게 바퀴 컨트롤러의 상태를 업데이트합니다.
   *
   * @param time_until_next   다음 상태 업데이트까지 예상되는 시간 간격 (seconds)
   * @param wheel_controller  운전 명령을 받을 바퀴 컨트롤러 객체
   */
  void updateWheelController(double time_until_next, SimpleWheelController &wheel_controller) override;
};

#endif  // DATA_INTEGRATE_TASKS_GOAL_POST_SEARCH_TASK_H
