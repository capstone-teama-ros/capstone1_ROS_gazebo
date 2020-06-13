#ifndef DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H
#define DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H

#include "../fixed_distance_driver.h"
#include "../stationary_align_driver.h"
#include "./task.h"

/**
 * 파란 공을 탐색하는 작업입니다.
 */
class BlueBallSearchTask : public Task
{
public:
  BlueBallSearchTask();

  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 현재 작업을 계속 진행해야 할 경우 @c nullptr 를 리턴하면 됩니다.
   * 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업의 객체를 리턴합니다.
   */
  TaskPtr tick(Blackboard &blackboard) override;

private:
  FixedDistanceDriver line_driver_;
  StationaryAlignDriver turn_driver_;
  bool has_started_ = false;
};

#endif  // DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H
