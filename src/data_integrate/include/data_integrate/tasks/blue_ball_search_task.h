#ifndef DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H
#define DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H

#include "./task.h"

/**
 * 파란 공을 탐색하는 작업입니다.
 */
class BlueBallSearchTask : public Task
{
public:
  const char *name() const override
  {
    return "BlueBallSearchTask";
  }

  BlueBallSearchTask();

private:
  TaskPtr search_task_;

  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 작업을 실행한 결과
   */
  TaskResult doTick(Blackboard &blackboard) override;

  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_BLUE_BALL_SEARCH_TASK_H
