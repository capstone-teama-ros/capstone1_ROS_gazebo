#include "data_integrate/blackboard.h"

#include <ros/ros.h>

double Blackboard::getRobotLinearSpeed() const
{
  // TODO: 진짜 선속도를 구해야 함
  return wheel_controller_.getLinearSpeed();
}

double Blackboard::getRobotAngularSpeed() const
{
  return visible_features_.getImuAngularVelocity().z;
}
