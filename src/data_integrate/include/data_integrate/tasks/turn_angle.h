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
   */
  TurnAngle(double angle) : angle_(angle){};

private:
  double angle_;
  double amount_turned_ = 0;
  bool is_first_tick_ = true;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_TURN_ANGLE_H
