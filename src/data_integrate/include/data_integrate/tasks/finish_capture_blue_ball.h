#ifndef DATA_INTEGRATE_TASKS_FINISH_CAPTURE_BLUE_BALL_H
#define DATA_INTEGRATE_TASKS_FINISH_CAPTURE_BLUE_BALL_H

#include "./task.h"

/**
 * Moves blindly in a straight line for a little bit, hoping to capture the blue ball in front of the robot.
 * If the blue ball is not captured after a while, aborts.
 */
class FinishCaptureBlueBall : public Task
{
public:
  const char *name() const override
  {
    return "FinishCaptureBlueBall";
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

  double move_timer_ = 0;
};

#endif  // DATA_INTEGRATE_TASKS_FINISH_CAPTURE_BLUE_BALL_H
