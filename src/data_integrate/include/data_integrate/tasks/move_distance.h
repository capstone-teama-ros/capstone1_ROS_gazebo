#ifndef DATA_INTEGRATE_TASKS_MOVE_DISTANCE_H
#define DATA_INTEGRATE_TASKS_MOVE_DISTANCE_H

#include "./task.h"

class MoveDistance : public Task
{
public:
  const char *name() const override
  {
    return "MoveDistance";
  }

  /// 최대 이동 속도 (m/s)
  static constexpr double MAX_SPEED = 0.5;
  /// 거리의 오차 범위 (meters)
  static constexpr double THRESHOLD = 0.01;

  /**
   * @param distance 이동할 거리 (meters)
   */
  MoveDistance(double distance) : distance_(distance){};

private:
  double distance_;
  double amount_moved_ = 0;
  double is_first_tick_ = true;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_MOVE_DISTANCE_H
