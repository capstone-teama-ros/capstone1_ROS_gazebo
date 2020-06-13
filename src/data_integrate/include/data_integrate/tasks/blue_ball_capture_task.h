#ifndef DATA_INTEGRATE_TASKS_BLUE_BALL_CAPTURE_TASK_H
#define DATA_INTEGRATE_TASKS_BLUE_BALL_CAPTURE_TASK_H

#include "./task.h"

/**
 * 이미 발견한 파란 공을 포푁하는 작업입니다.
 */
class BlueBallCaptureTask : public Task
{
public:
  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 현재 작업을 계속 진행해야 할 경우 @c nullptr 를 리턴하면 됩니다.
   * 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업의 객체를 리턴합니다.
   */
  TaskPtr tick(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_BLUE_BALL_CAPTURE_TASK_H
