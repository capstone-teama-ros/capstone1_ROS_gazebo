#include "../../include/features/rel_point.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include <cmath>

// 참고: 각도는 y축을 기준으로 반시계 방향으로 측정합니다.
// 따라서 x는 r * cos(theta) 가 아니라 r * (-sin(theta))가 됩니다.
// 또한   y는 r * sin(theta) 가 아니라 r * cos(theta)가 됩니다.
RelPoint::RelPoint(double distance, double angle)
  : RelPoint(distance, angles::normalize_angle(angle), distance * -std::sin(angle), distance * std::cos(angle))
{
  ROS_ASSERT(distance >= 0);
}

RelPoint RelPoint::fromRelXY(double rel_x, double rel_y)
{
  double distance = std::hypot(rel_x, rel_y);
  double angle = 0;  // 기본값
  if (distance > 0)
  {
    // 0 ~ π 사이의 값을 만들기 위해 atan()이 아닌 acos()를 사용함
    angle = std::acos(rel_y / distance);
  }

  return RelPoint(distance, angle, rel_x, rel_y);
}
