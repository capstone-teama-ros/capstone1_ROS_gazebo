#include "data_integrate/tasks/line_tracer.h"

#include <ros/ros.h>

void LineTracer::doHalt(Blackboard &blackboard)
{
  blackboard.useSimpleWheelController = true;
}

TaskResult LineTracer::doTick(Blackboard &blackboard)
{
  blackboard.useSimpleWheelController = false;

  auto &vfm = blackboard.visible_features_;
  auto &direct_controller = blackboard.direct_controller_;
  double lt_box_midpoint = vfm.getTracerBoxX() + vfm.getTracerBoxWidth() / 2.0;
  double lt_image_width = vfm.getTracerImageWidth();

  // midpoint의 상대적 위치를 -1 ~ 1 사이의 값으로 변환한다.
  double midpoint_relative = lt_box_midpoint / lt_image_width * 2 - 1;
  // Fix for initial value
  if (lt_box_midpoint == 0)
  {
    midpoint_relative = 0;
  }

  // 속도 파라미터
  double speed_base;
  double speed_turn_factor;
  ros::param::param<double>("~speed_base", speed_base, 0.6);
  ROS_INFO_ONCE("Reading parameter from: %s", ros::names::resolve("~speed_base").c_str());
  ros::param::param<double>("~speed_turn_factor", speed_turn_factor, 0.6);
  ROS_INFO_ONCE("Reading parameter from: %s", ros::names::resolve("~speed_turn_factor").c_str());
  speed_base /= 0.063;
  speed_turn_factor /= 0.063;

  auto left_speed = speed_base + speed_turn_factor * midpoint_relative;
  auto right_speed = speed_base - speed_turn_factor * midpoint_relative;

  direct_controller.setWheelSpeeds(left_speed, right_speed);
  ROS_INFO("midpoint_relative = %.3f, left_speed = %.3f, right_speed = %.3f", midpoint_relative, left_speed,
           right_speed);

  // TODO: 일단은 Line Tracer 모드의 종료 조건을 안 정했다. 나중에 정하자.
  return TaskResult::Running;
}
