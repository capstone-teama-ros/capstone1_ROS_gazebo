#ifndef DATA_INTEGRATE_TASKS_TURN_TO_NEAREST_BALL_H
#define DATA_INTEGRATE_TASKS_TURN_TO_NEAREST_BALL_H

#include "./task.h"

#include "../features/ball.h"

/**
 * Aligns the robot's direction to the nearest ball of a specific color.
 */
class TurnToNearestBall : public Task
{
public:
  /**
   * @param ball_color Color of the ball to find a clear path to
   */
  TurnToNearestBall(BallColor ball_color) : ball_color_(ball_color)
  {
  }

  const char *name() const override;

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

  BallColor ball_color_;
  double current_finish_timer_ = 0;
};

#endif  // DATA_INTEGRATE_TASKS_TURN_TO_NEAREST_BALL_H
