#include <core_msgs/ball_position.h>
#include <core_msgs/line_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include "data_integrate/blackboard.h"
#include "data_integrate/features/visible_feature_manager.h"
#include "data_integrate/simple_wheel_controller.h"
#include "data_integrate/task_executor.h"

int section;

void camera_Callback_2(const core_msgs::line_info::ConstPtr& line_section)
{
  //is_bump = line_section->is_bump;
  section = line_section->section;
}

enum class CameraLinePosition : int32_t
{
  NONE = 3,
  //FAR_LEFT = 1,
  LEFT = 0,
  CENTER = 1,
  RIGHT = 2,
  //FAR_RIGHT = 5,
};


enum class TurnDirection
{
  LEFT,
  RIGHT,
};

const double LINEAR_SPEED_FAST = 0.5;
const double LINEAR_SPEED_SLOW = 0.35;
const double ANGULAR_SPEED_FAST = 1;
const double ANGULAR_SPEED_SLOW = 0.2;

/**
 * 라인트레이서 모드 실행
 */
void updateLineTracerState(SimpleWheelController& wheel_controller, CameraLinePosition cameraLinePosition,
                           TurnDirection& last_turn_direction)
{
  double linear_speed = 0;
  double angular_speed = 0;

  switch (cameraLinePosition)
  {
    case CameraLinePosition::CENTER:
      wheel_controller.moveLinear(LINEAR_SPEED_SLOW);
      wheel_controller.turn(0);
      break;
    case CameraLinePosition::LEFT:
      last_turn_direction = TurnDirection::LEFT;
      wheel_controller.moveLinear(LINEAR_SPEED_SLOW);
      wheel_controller.turn(ANGULAR_SPEED_SLOW);
      break;
    case CameraLinePosition::RIGHT:
      last_turn_direction = TurnDirection::RIGHT;
      wheel_controller.moveLinear(LINEAR_SPEED_SLOW);
      wheel_controller.turn(-ANGULAR_SPEED_SLOW);
      break;
    // case CameraLinePosition::FAR_LEFT:
    //   last_turn_direction = TurnDirection::LEFT;
    //   // is_bump에 상관없이 선속도는 느리게 유지한다.
    //   linear_speed = LINEAR_SPEED_SLOW;
    //   angular_speed = ANGULAR_SPEED_FAST;
    //   break;
    // case CameraLinePosition::FAR_RIGHT:
    //   last_turn_direction = TurnDirection::RIGHT;
    //   // is_bump에 상관없이 선속도는 느리게 유지한다.
    //   linear_speed = LINEAR_SPEED_SLOW;
    //   angular_speed = -ANGULAR_SPEED_FAST;
    //   break;
    case CameraLinePosition::NONE:
      // 로봇이 검은 줄을 놓쳤다.
      // is_bump에 상관없이 선속도는 느리게 유지한다.
      //linear_speed_ = LINEAR_SPEED_SLOW;
      wheel_controller.moveLinear(LINEAR_SPEED_SLOW);
      // 가장 최근에 회전했던 방향의 반대방향으로 지속적으로 회전하면서 검은 줄을 찾아보자.
      if (last_turn_direction == TurnDirection::RIGHT)
        wheel_controller.turn(ANGULAR_SPEED_FAST);
      else
        wheel_controller.turn(-ANGULAR_SPEED_FAST);
      break;
    default:
      std::cerr << "[LOGIC ERROR] Unknown camera line position: " << static_cast<int>(cameraLinePosition) << std::endl;
      break;
  }

  //wheel_controller.publish(linear_speed, angular_speed);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integration_node");
  ros::NodeHandle n;

  VisibleFeatureManager visible_features;

  ros::Subscriber sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &VisibleFeatureManager::subscribeToLidar, &visible_features);
  ros::Subscriber sub_ball_harvest = n.subscribe<core_msgs::ball_position>(
      "/position", 1000, &VisibleFeatureManager::subscribeToCamera, &visible_features);
  ros::Subscriber sub_line_tracing = 
      n.subscribe<core_msgs::line_info>("/line_section", 1000, camera_Callback_2);
  ros::Subscriber sub_imu =
      n.subscribe<sensor_msgs::Imu>("/imu", 1000, &VisibleFeatureManager::subscribeToImu, &visible_features);

  ros::Publisher fl_wheel = n.advertise<std_msgs::Float64>("/myrobot/FLwheel_velocity_controller/command", 10);
  ros::Publisher fr_wheel = n.advertise<std_msgs::Float64>("/myrobot/FRwheel_velocity_controller/command", 10);
  ros::Publisher fl_publish = n.advertise<std_msgs::Float64>("myrobot/FLsuspension_position_controller/command", 10);
  ros::Publisher fr_publish = n.advertise<std_msgs::Float64>("/myrobot/FRsuspension_position_controller/command", 10);
  ros::Publisher cs_publish = n.advertise<std_msgs::Float64>("/myrobot/CSsuspension_position_controller/command", 10);

  SimpleWheelController wheel_controller(fl_wheel, fr_wheel);
  Blackboard blackboard(visible_features, wheel_controller);
  TaskExecutor task_executor(blackboard);

  auto last_turn_direction = TurnDirection::LEFT;
  ros::Duration sleep_duration(0.025);

  while (ros::ok())
  {
    std_msgs::Float64 FL_position_msg;
    std_msgs::Float64 FR_position_msg;
    std_msgs::Float64 CS_position_msg;

    FL_position_msg.data = 0.0;
    FR_position_msg.data = 0.0;
    CS_position_msg.data = 0.0;

    fl_publish.publish(FL_position_msg);
    fr_publish.publish(FR_position_msg);
    cs_publish.publish(CS_position_msg);

    // TODO 더 정확한 시간을 사용하기
    //task_executor.runTaskInLoop(sleep_duration.toSec(), sleep_duration.toSec());
    updateLineTracerState(wheel_controller, static_cast<CameraLinePosition>(section), last_turn_direction);
    wheel_controller.publish();
    sleep_duration.sleep();
    ros::spinOnce();
  }

  return 0;
}
