#include "data_integrate/tasks/line_tracer.h"

#include <ros/ros.h>

void LineTracer::doHalt(Blackboard &blackboard)
{
  line_position_ = LinePosition::None;
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

  if (midpoint_relative < -0.2)
  {
    line_position_ = LinePosition::Left;
  }
  else if (midpoint_relative <= 0.2)
  {
    line_position_ = LinePosition::Center;
  }
  else
  {
    line_position_ = LinePosition::Right;
  }

  // 속도 파라미터
  double linear_speed;
  double linear_speed_slow;
  double angular_speed_bonus;
  ros::param::param<double>("~linear_speed", linear_speed, 0);
  ros::param::param<double>("~linear_speed_slow", linear_speed_slow, 0);
  ros::param::param<double>("~angular_speed_bonus", angular_speed_bonus, 0);
  linear_speed /= 0.063;
  linear_speed_slow /= 0.063;
  angular_speed_bonus /= 0.063;

  switch (line_position_)
  {
    case LinePosition::Center:
      direct_controller.setWheelSpeeds(linear_speed, linear_speed);
      ROS_INFO("Center (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::Left:
      direct_controller.setWheelSpeeds(linear_speed_slow - angular_speed_bonus,
                                       linear_speed_slow + angular_speed_bonus);
      ROS_INFO("Left (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::Right:
      direct_controller.setWheelSpeeds(linear_speed_slow + angular_speed_bonus,
                                       linear_speed_slow - angular_speed_bonus);
      ROS_INFO("Right (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::None:
      // 로봇이 검은 줄을 놓쳤다.
      ROS_WARN("No line found!");
      direct_controller.setWheelSpeeds(linear_speed, linear_speed);
      break;
    default:
      ROS_ASSERT_MSG(0, "Unexpected line position: %u", static_cast<unsigned int>(line_position_));
      break;
  }

  // TODO: 일단은 Line Tracer 모드의 종료 조건을 안 정했다. 나중에 정하자.
  return TaskResult::Running;
}
