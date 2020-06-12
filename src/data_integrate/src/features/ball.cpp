#include "../../include/features/ball.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include <cmath>

// 참고: 각도는 y축을 기준으로 반시계 방향으로 측정합니다.
// 따라서 x는 r * cos(theta) 가 아니라 r * (-sin(theta))가 됩니다.
// 또한   y는 r * sin(theta) 가 아니라 r * cos(theta)가 됩니다.
Ball::Ball(BallColor color, double distance, double angle) : color_(color), position_(distance, angle)
{
}

Ball Ball::fromRelXY(BallColor color, double rel_x, double rel_y)
{
  return Ball(color, RelPoint::fromRelXY(rel_x, rel_y));
}

double Ball::getRelX() const
{
  return position_.getRelX();
}

double Ball::getRelY() const
{
  return position_.getRelY();
}

double Ball::getDistance() const
{
  return position_.getDistance();
}

double Ball::getAngle() const
{
  return position_.getAngle();
}
