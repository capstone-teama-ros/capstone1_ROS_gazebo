#ifndef DATA_INTEGRATE_TASKS_KICK_BALL_INTO_GOAL_H
#define DATA_INTEGRATE_TASKS_KICK_BALL_INTO_GOAL_H

#include "./task.h"

/**
 * Moves in a straight line until the goal (green ball) is very near, and then stops.
 * Fails if the robot is not aligned with the goal, or if blue ball is not ejected.
 */
class KickBallIntoGoal : public Task
{
public:
  const char *name() const override
  {
    return "KickBallIntoGoal";
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

  double kick_confirm_timer_;
};

#endif  // DATA_INTEGRATE_TASKS_KICK_BALL_INTO_GOAL_H
