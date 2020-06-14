#ifndef DATA_INTEGRATE_TASKS_TURN_ANGLE_H
#define DATA_INTEGRATE_TASKS_TURN_ANGLE_H

#include "./task.h"

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
   * @param angle 회전할 각도 (radians)
   * @param max_turn_speed 최대 회전 속도 (rad/s). 0 이상의 값이어야 합니다. 기본값은 MAX_TURN_SPEED입니다.
   */
  TurnAngle(double angle, double max_turn_speed = MAX_TURN_SPEED) : angle_(angle), max_turn_speed_(max_turn_speed){};

private:
  double angle_;
  double max_turn_speed_;
  double amount_turned_ = 0;
  bool is_first_tick_ = true;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_TURN_ANGLE_H
