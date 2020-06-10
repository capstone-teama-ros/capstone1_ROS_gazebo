#include "../include/fixed_distance_driver.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>

FixedDistanceDriver::FixedDistanceDriver(double max_speed, double min_speed, double distance_threshold)
  : max_speed_(max_speed), min_speed_(min_speed), distance_threshold_(distance_threshold)
{
}

void FixedDistanceDriver::setDistance(double distance)
{
  distance_ = distance;
}

void FixedDistanceDriver::updateController(SimpleWheelController& wheel_controller, double time_to_next_update)
{
  ROS_ASSERT(max_speed_ > 0);
  ROS_ASSERT(min_speed_ < 0);
  ROS_ASSERT(distance_threshold_ >= 0);

  ROS_ASSERT(time_to_next_update > 0);

  // 남은 거리가 오차 범위 이내이면 도달한 것으로 취급하여 정지합니다.
  if (std::abs(distance_) <= distance_threshold_)
  {
    wheel_controller.stop();
    return;
  }

  double speed;
  if (distance_ > 0)
  {
    // 전진 속도 계산
    speed = std::min(distance_ / time_to_next_update, max_speed_);
  }
  else
  {
    // 후진 속도 계산
    speed = std::max(distance_ / time_to_next_update, min_speed_);
  }
  wheel_controller.moveLinear(speed);
  distance_ -= speed * time_to_next_update;
}
