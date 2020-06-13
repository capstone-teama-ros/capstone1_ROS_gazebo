#ifndef DATA_INTEGRATE_BLACKBOARD_H
#define DATA_INTEGRATE_BLACKBOARD_H

#include "./features/past_feature_manager.h"
#include "./features/visible_feature_manager.h"
#include "./simple_wheel_controller.h"

/**
 * 로봇의 모든 상태를 저장하고 관리하는 클래스입니다.
 */
class Blackboard
{
public:
  /// 과거의 지형지물의 신뢰성이 감소하는 속도
  /// TODO 값을 조절할 수 있게 만들어야 함
  static constexpr double PAST_FEATURE_DECAY_RATE = 0.8;

  Blackboard(VisibleFeatureManager &visible_features, SimpleWheelController &wheel_controller)
    : visible_features_(visible_features)
    , past_features_(PAST_FEATURE_DECAY_RATE)
    , wheel_controller_(wheel_controller){};

  VisibleFeatureManager &visible_features_;
  PastFeatureManager past_features_;
  SimpleWheelController &wheel_controller_;

  double getTimeSinceLastTick() const
  {
    return time_since_last_tick_;
  }

  double setTimeSinceLastTick(double time_since_last_tick)
  {
    time_since_last_tick_ = time_since_last_tick;
  }

  double getTimeUntilNextTick() const
  {
    return time_until_next_tick_;
  }

  double setTimeUntilNextTick(double time_until_next_tick)
  {
    time_until_next_tick_ = time_until_next_tick;
  }

private:
  double time_since_last_tick_ = 0;
  double time_until_next_tick_ = 0;
};

#endif  // DATA_INTEGRATE_BLACKBOARD_H
