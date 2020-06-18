#ifndef DATA_INTEGRATE_TASKS_TURN_ANGLE_H
#define DATA_INTEGRATE_TASKS_TURN_ANGLE_H

#include "./task.h"

/**
 * Rotates the robot for a given amount of angle, optionally with a small, constant linear velocity component.
 */
class TurnAngle : public Task
{
public:
  const char *name() const override
  {
    return "TurnAngle";
  }

  /// 최대 회전 속도 (rad/s)
  static const double MAX_TURN_SPEED;
  /// 각도의 오차 범위 (radians)
  static const double THRESHOLD;

  /**
   * @param angle           Amount of angle to turn (radians)
   * @param max_turn_speed  Maximum turn speed. Must be 0 or greater. (rad/s)
   * @param linear_speed   Linear velocity amount (m/s)
   */
  TurnAngle(double angle, double max_turn_speed = MAX_TURN_SPEED, double linear_speed = 0)
    : angle_(angle), max_turn_speed_(max_turn_speed), linear_speed_(linear_speed)
  {
  }

private:
  double angle_;
  double max_turn_speed_;
  double linear_speed_;
  double amount_turned_ = 0;
  bool is_first_tick_ = true;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_TURN_ANGLE_H
