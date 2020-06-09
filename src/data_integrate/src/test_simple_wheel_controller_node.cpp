/**
 * @file SimpleWheelController를 테스트하기 위한 노드입니다.
 */

#include <cmath>

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "../include/simple_wheel_controller.h"

/**
 * 현재 로봇이 실행하고 있는 동작 상태를 나타냅니다.
 */
enum class MoveActionState
{
  Stop,  ///< 정지한 상태
  Line,  ///< 정사각형의 변을 따라 직진한다.
  Turn,  ///< 정사각형의 꼭짓점에서 회전한다.
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_simple_wheel_controller");
  ros::NodeHandle n("~");

  ros::Publisher left_wheel = n.advertise<std_msgs::Float64>("left_wheel", 10);
  ros::Publisher right_wheel = n.advertise<std_msgs::Float64>("right_wheel", 10);

  SimpleWheelController wheel_controller(left_wheel, right_wheel);

  int action_counter = 0;
  int line_segment_counter = 0;
  auto action_state = MoveActionState::Stop;

  const double PUBLISH_INTERVAL = 0.025;
  ros::Duration publish_interval(PUBLISH_INTERVAL);

  // 운전 관련 상수
  const double LINE_SPEED = 0.5;   // m/s
  const double LINE_LENGTH = 1.0;  // meters
  const double TURN_SPEED = angles::from_degrees(30);
  const double TURN_ANGLE = angles::from_degrees(90);
  const double STOP_DURATION = 1.5;  // seconds

  ROS_INFO("SimpleWheelController test node is publishing on:\n  %s\n  %s", left_wheel.getTopic().c_str(),
           right_wheel.getTopic().c_str());

  while (ros::ok())
  {
    // 테스트용 코드
    // 로봇이 직진 -> 반시계 회전 -> 직진 -> 반시계 회전 -> ... 을 반복하며 가로, 세로 1m의 정사각형을 그리게 합니다.
    // 처음 위치에 돌아오면 1초간 정지합니다.

    // 현재 상태를 유지하는 카운터 감소
    --action_counter;

    // 카운터가 0이 되면 다음 상태로 전환
    if (action_counter <= 0)
    {
      switch (action_state)
      {
        case MoveActionState::Stop:
          action_state = MoveActionState::Line;
          action_counter = std::round(LINE_LENGTH / LINE_SPEED / PUBLISH_INTERVAL);
          ROS_INFO("Moving straight for %f second(s)", LINE_LENGTH / LINE_SPEED);
          break;
        case MoveActionState::Line:
          action_state = MoveActionState::Turn;
          action_counter = std::round(TURN_ANGLE / TURN_SPEED / PUBLISH_INTERVAL);
          ROS_INFO("Turning left for %f second(s)", TURN_ANGLE / TURN_SPEED);
          break;
        case MoveActionState::Turn:
          // 4번째 변까지 그렸다면 잠시 정지
          if (line_segment_counter >= 3)
          {
            action_state = MoveActionState::Stop;
            action_counter = std::round(STOP_DURATION / PUBLISH_INTERVAL);
            line_segment_counter = 0;
            ROS_INFO("Stopping for %f second(s)", STOP_DURATION);
          }
          else
          {
            action_state = MoveActionState::Line;
            action_counter = std::round(LINE_LENGTH / LINE_SPEED / PUBLISH_INTERVAL);
            ++line_segment_counter;
            ROS_INFO("Moving straight for %f second(s)", LINE_LENGTH / LINE_SPEED);
          }
          break;
        default:
          ROS_ASSERT_MSG(0, "Invalid action state: %d", action_state);
      }
    }

    // 현재 상태에 맞게 바퀴 속도를 설정
    switch (action_state)
    {
      case MoveActionState::Stop:
        wheel_controller.stop();
        break;
      case MoveActionState::Line:
        wheel_controller.moveLinear(LINE_SPEED);
        break;
      case MoveActionState::Turn:
        wheel_controller.turn(TURN_SPEED);
        break;
      default:
        ROS_ASSERT_MSG(0, "Invalid action state: %d", action_state);
    }

    wheel_controller.publish();
    publish_interval.sleep();
  }

  return 0;
}
