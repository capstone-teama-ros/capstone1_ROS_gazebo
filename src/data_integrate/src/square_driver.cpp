#include "data_integrate/square_driver.h"

#include <angles/angles.h>

void SquareDriver::updateState(double time_passed)
{
  action_timer_ -= time_passed;
  if (action_timer_ > 0)
    return;
  // 타이머가 0이 되면 다음 운행 상태로 전환합니다.

  ROS_ASSERT(line_speed_ > 0);
  ROS_ASSERT(turn_speed_ > 0);
  ROS_ASSERT(rest_duration_ > 0);

  // 직진 -> 반시계 회전 -> 직진 -> 반시계 회전 -> ... 을 반복하며 정사각형을 그립니다.
  // 처음 위치에 돌아오면 잠시 정지합니다.
  switch (action_state_)
  {
    case MoveActionState::Stop:
      action_state_ = MoveActionState::Line;
      action_timer_ = side_length_ / line_speed_;
      ROS_INFO("Moving straight for %f second(s)", action_timer_);
      break;
    case MoveActionState::Line:
      action_state_ = MoveActionState::Turn;
      action_timer_ = angles::from_degrees(90) / turn_speed_;
      ROS_INFO("Turning left for %f second(s)", action_timer_);
      break;
    case MoveActionState::Turn:
      // 4번째 변까지 그렸다면 잠시 정지
      if (line_segment_counter_ >= 3)
      {
        action_state_ = MoveActionState::Stop;
        action_timer_ = rest_duration_;
        line_segment_counter_ = 0;
        ROS_INFO("Stopping for %f second(s)", rest_duration_);
      }
      else
      {
        action_state_ = MoveActionState::Line;
        action_timer_ = side_length_ / line_speed_;
        ++line_segment_counter_;
        ROS_INFO("Moving straight for %f second(s)", action_timer_);
      }
      break;
    default:
      ROS_ASSERT_MSG(0, "Invalid action state: %u", static_cast<unsigned int>(action_state_));
  }
}

void SquareDriver::updateController(SimpleWheelController& wheel_controller) const
{
  switch (action_state_)
  {
    case MoveActionState::Stop:
      wheel_controller.stop();
      break;
    case MoveActionState::Line:
      wheel_controller.moveLinear(line_speed_);
      break;
    case MoveActionState::Turn:
      wheel_controller.turn(turn_speed_);
      break;
    default:
      ROS_ASSERT_MSG(0, "Invalid action state: %d", static_cast<unsigned int>(action_state_));
  }
}
