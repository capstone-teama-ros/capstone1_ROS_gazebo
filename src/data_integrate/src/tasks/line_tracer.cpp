#include "data_integrate/tasks/line_tracer.h"

#include <ros/ros.h>

void LineTracer::doHalt(Blackboard &blackboard)
{
  line_position_ = LinePosition::None;
  blackboard.useSimpleWheelController = true;
}

TaskResult LineTracer::doTick(Blackboard &blackboard)
{
  double linear_speed = 0;
  double angular_speed = 0;

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
  const double LINEAR_SPEED = 0.35;
  const double ANGULAR_SPEED_BONUS = 0.2;

  switch (line_position_)
  {
    case LinePosition::Center:
      direct_controller.setWheelSpeeds(LINEAR_SPEED, LINEAR_SPEED);
      ROS_INFO("Center (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::Left:
      direct_controller.setWheelSpeeds(LINEAR_SPEED - ANGULAR_SPEED_BONUS, LINEAR_SPEED + ANGULAR_SPEED_BONUS);
      ROS_INFO("Left (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::Right:
      direct_controller.setWheelSpeeds(LINEAR_SPEED + ANGULAR_SPEED_BONUS, LINEAR_SPEED - ANGULAR_SPEED_BONUS);
      ROS_INFO("Right (midpoint_relative = %.3f)", midpoint_relative);
      break;
    case LinePosition::None:
      // 로봇이 검은 줄을 놓쳤다.
      ROS_WARN("No line found!");
      direct_controller.setWheelSpeeds(LINEAR_SPEED, LINEAR_SPEED);
      break;
    default:
      ROS_ASSERT_MSG(0, "Unexpected line position: %u", static_cast<unsigned int>(line_position_));
      break;
  }

  // TODO: 일단은 Line Tracer 모드의 종료 조건을 안 정했다. 나중에 정하자.
  return TaskResult::Running;
}
