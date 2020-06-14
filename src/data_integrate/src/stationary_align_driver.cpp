#include "data_integrate/stationary_align_driver.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>

StationaryAlignDriver::StationaryAlignDriver(double max_turn_speed, double angle_threshold, double distance_threshold)
  : max_turn_speed_(max_turn_speed), angle_threshold_(angle_threshold), distance_threshold_(distance_threshold)
{
}

void StationaryAlignDriver::updatePoint(double rel_x, double rel_y)
{
  rel_x_ = rel_x;
  rel_y_ = rel_y;
}

void StationaryAlignDriver::updateController(SimpleWheelController& wheel_controller, double time_to_next_update) const
{
  ROS_ASSERT(max_turn_speed_ >= 0);
  ROS_ASSERT(angle_threshold_ >= 0);
  ROS_ASSERT(distance_threshold_ >= 0);

  ROS_ASSERT(time_to_next_update > 0);

  // 점이 매우 가까우면 도달한 것으로 취급하여 정지합니다.
  auto distance = std::hypot(rel_x_, rel_y_);
  if (distance <= distance_threshold_)
  {
    wheel_controller.stop();
    return;
  }

  // 점이 자신의 정면 방향과 거의 일치하면 정지합니다.
  ROS_ASSERT(distance > 0);
  auto angle_difference = std::acos(rel_y_ / distance);
  if (angle_difference <= angle_threshold_)
  {
    wheel_controller.stop();
    return;
  }

  // 알맞은 회전 속도를 계산합니다.
  auto turn_speed = std::min(angle_difference / time_to_next_update, max_turn_speed_);
  // 점이 y축을 기준으로 오른쪽에 있으면 시계 방향으로 회전합니다.
  if (rel_x_ > 0)
  {
    turn_speed = -turn_speed;
  }
  wheel_controller.turn(turn_speed);
}
