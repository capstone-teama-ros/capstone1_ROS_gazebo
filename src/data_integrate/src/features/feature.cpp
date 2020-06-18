#include "data_integrate/features/feature.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include <cmath>

// Note: Angles are measured from the y axis, counterclockwise.
// Thus, x == r * -sin(theta) rather than r * cos(theta).
// Also, y == r * cos(theta) rather than r * sin(theta).
Feature::Feature(double distance, double angle)
  : distance_(distance)
  , angle_(angles::normalize_angle(angle))
  , rel_x_(distance * -std::sin(angle))
  , rel_y_(distance * std::cos(angle))
{
  ROS_ASSERT(distance >= 0);
}

void Feature::setRelXY(Feature *feature, double rel_x, double rel_y)
{
  feature->rel_x_ = rel_x;
  feature->rel_y_ = rel_y;
  feature->distance_ = std::hypot(rel_x, rel_y);
  if (feature->distance_ == 0)
  {
    feature->angle_ = 0;
  }
  else
  {
    feature->angle_ = std::atan2(-rel_x, rel_y);
  }
}

double Feature::getDistanceTo(const Feature &other) const
{
  return std::hypot(getRelX() - other.getRelX(), getRelY() - other.getRelY());
}
