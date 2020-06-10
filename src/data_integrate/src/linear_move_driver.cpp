#include "../include/linear_move_driver.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>

LinearMoveDriver::LinearMoveDriver(double linear_speed, double align_threshold, double distance_threshold)
  : linear_speed_(linear_speed), align_threshold_(align_threshold), distance_threshold_(distance_threshold)
{
}


void LinearMoveDriver::updatePoint(double rel_x, double rel_y){
  rel_x_ = rel_x;
  rel_y_ = rel_y;
}



void LinearMoveDriver::updateController(SimpleWheelController& wheel_controller) const{

  auto distance = std::hypot(rel_x_, rel_y_);

  // 줍고자 하는 공이랑 align이 잘 되어있는 확인!

  if (std::abs(std::atan(rel_x/rel_y)) > align_threshold_){
    wheel_controller.stop();
    return;
  }

  // 목표지점까지 가는 와중에 목표물과 일정거리 이하라면 멈추고 align 확인합니다.
  if(distance <= distance_threshold_){
    wheel_controller.stop();
    return;
  }

  wheel_controller.moveLinear(linear_speed_);
}
