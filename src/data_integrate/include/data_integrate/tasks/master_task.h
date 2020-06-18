#ifndef DATA_INTEGRATE_TASKS_MASTER_TASK_H
#define DATA_INTEGRATE_TASKS_MASTER_TASK_H

#include "./task.h"

#include "./ball_harvester.h"
#include "./line_tracer.h"

/**
 * Master task for the robot
 */
class MasterTask : public Task
{
public:
  MasterTask();

  const char *name() const override
  {
    return "MasterTask";
  }

private:
  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 작업을 실행한 결과
   */
  TaskResult doTick(Blackboard &blackboard) override;

  void doHalt(Blackboard &blackboard) override;

  LineTracer line_tracer_task_;
  BallHarvester ball_harvester_task_;
  bool is_line_tracer_ = true;
};

#endif  // DATA_INTEGRATE_TASKS_MASTER_TASK_H
