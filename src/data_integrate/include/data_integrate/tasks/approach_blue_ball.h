#ifndef DATA_INTEGRATE_TASKS_APPROACH_BLUE_BALL_H
#define DATA_INTEGRATE_TASKS_APPROACH_BLUE_BALL_H

#include "./task.h"

/**
 * Approaches the blue ball until it is "close enough".
 * Aborts if there is no blue ball in a straight line in front of the robot.
 */
class ApproachBlueBall : public Task
{
public:
  const char *name() const override
  {
    return "ApproachBlueBall";
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
};

#endif  // DATA_INTEGRATE_TASKS_APPROACH_BLUE_BALL_H
