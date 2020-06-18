#ifndef DATA_INTEGRATE_TASKS_AVOID_OBSTACLES_TO_BALL_H
#define DATA_INTEGRATE_TASKS_AVOID_OBSTACLES_TO_BALL_H

#include "./task.h"

#include <deque>
#include <string>
#include "../features/ball.h"
#include "./turn_to_nearest_ball.h"

/**
 * If the robot is aligned with a ball of specific color, moves to a new position, then re-aligns until the robot has a
 * clear path to the target ball.
 *
 * Succeeds if the task manages to acquire a clear path to the target-colored ball.
 * Fails if any avoidance subtasks fail, or if there are no target balls visible.
 */
class AvoidObstaclesToBall : public Task
{
public:
  /**
   * @param ball_color Color of the ball to find a clear path to
   */
  AvoidObstaclesToBall(BallColor ball_color) : ball_color_(ball_color), align_task_(ball_color)
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
  TurnToNearestBall align_task_;
  std::deque<TaskPtr> queued_tasks_;
};

#endif  // DATA_INTEGRATE_TASKS_APPROACH_BLUE_BALL_H
