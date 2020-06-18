#include "data_integrate/tasks/turn_angle.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

const double TurnAngle::MAX_TURN_SPEED = angles::from_degrees(30);
const double TurnAngle::THRESHOLD = angles::from_degrees(0.5);

void TurnAngle::doHalt(Blackboard &blackboard)
{
  blackboard.wheel_controller_.stop();
  amount_turned_ = 0;
  is_first_tick_ = true;
}

TaskResult TurnAngle::doTick(Blackboard &blackboard)
{
  ROS_ASSERT(max_turn_speed_ >= 0);
  ROS_ASSERT(THRESHOLD >= 0);

  if (is_first_tick_)
  {
    is_first_tick_ = false;
  }
  else
  {
    // 회전한 각도를 업데이트
    // TODO: 실제 회전한 각도를 사용할 수 있으면 좋겠다.
    amount_turned_ += blackboard.getTimeSinceLastTick() * blackboard.getRobotAngularSpeed();
  }

  // 원하는 각도까지 회전했으면 성공
  auto angle_difference = std::abs(angle_ - amount_turned_);
  if (angle_difference <= THRESHOLD)
  {
    halt(blackboard);
    return TaskResult::Success;
  }

  // 알맞은 회전 속도를 계산합니다.
  auto turn_speed = std::min(angle_difference / blackboard.getTimeUntilNextTick(), max_turn_speed_);
  if (angle_ < 0)
  {
    turn_speed = -turn_speed;
  }
  blackboard.wheel_controller_.moveComposite(linear_speed_, turn_speed);
  return TaskResult::Running;
}
