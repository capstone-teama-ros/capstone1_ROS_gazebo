/**
 * @file SimpleWheelController를 테스트하기 위한 노드입니다.
 */

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "data_integrate/simple_wheel_controller.h"

/**
 * 현재 로봇이 실행하고 있는 운행 상태를 나타냅니다.
 */
enum class MoveActionState
{
  Stop,  ///< 정지한 상태
  Line,  ///< 정사각형의 변을 따라 직진한다.
  Turn,  ///< 정사각형의 꼭짓점에서 회전한다.
};

/**
 * 반시계 방향으로 정사각형을 그리는 운전 알고리즘. 정사각형을 모두 그리면 잠시 쉬었다가 다시 시작합니다.
 */
class SquareDriver
{
public:
  /**
   * @param side_length   한 변의 길이 (meters)
   * @param line_speed    한 변을 따라 이동할 때의 선속도 (m/s)
   * @param turn_speed    꼭짓점에서 회전할 때의 각속도 (rad/s)
   * @param rest_duration 정사각형을 1번 그린 후 쉬는 시간 (seconds)
   */
  SquareDriver(double side_length, double line_speed, double turn_speed, double rest_duration);

  /**
   * 현재 운행 상태를 시간이 @p time_passed 만큼 지나간 후의 상태로 변경합니다.
   * @param time_passed 지나간 시간
   */
  void updateState(double time_passed);

  /**
   * 현재 운행 상태에 알맞게 바퀴 컨트롤러를 업데이트합니다.
   */
  void updateController(SimpleWheelController& wheel_controller) const;

private:
  double side_length_ = 1;
  double line_speed_ = 1;
  double turn_speed_ = 1;
  double rest_duration_ = 1;
  double action_timer_ = 0;
  double line_segment_counter_ = 0;
  MoveActionState action_state_ = MoveActionState::Stop;
};

SquareDriver::SquareDriver(double side_length, double line_speed, double turn_speed, double rest_duration)
  : side_length_(side_length), line_speed_(line_speed), turn_speed_(turn_speed), rest_duration_(rest_duration)
{
  // 비워놓음
}

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
      ROS_ASSERT_MSG(0, "Invalid action state: %u", static_cast<unsigned int>(action_state_));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_simple_wheel_controller");
  ros::NodeHandle n("~");

  ros::Publisher left_wheel = n.advertise<std_msgs::Float64>("left_wheel", 10);
  ros::Publisher right_wheel = n.advertise<std_msgs::Float64>("right_wheel", 10);

  SimpleWheelController wheel_controller(left_wheel, right_wheel);

  ros::Duration publish_interval(0.025);  // seconds

  const double SIDE_LENGTH = 1.0;                      // meters
  const double LINE_SPEED = 0.5;                       // m/s
  const double TURN_SPEED = angles::from_degrees(90);  // rad/s
  const double REST_DURATION = 1.5;                    // seconds
  SquareDriver driver(SIDE_LENGTH, LINE_SPEED, TURN_SPEED, REST_DURATION);

  ROS_INFO("SimpleWheelController test node is publishing on:\n  %s\n  %s", left_wheel.getTopic().c_str(),
           right_wheel.getTopic().c_str());

  while (ros::ok())
  {
    driver.updateState(publish_interval.toSec());
    driver.updateController(wheel_controller);
    wheel_controller.publish();

    publish_interval.sleep();
  }

  return 0;
}
