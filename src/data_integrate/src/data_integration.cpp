#include <angles/angles.h>
#include <core_msgs/ball_position.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vector>

#include "data_integrate/blackboard.h"
#include "data_integrate/features/visible_feature_manager.h"
#include "data_integrate/simple_wheel_controller.h"
#include "data_integrate/task_executor.h"
#include "data_integrate/tasks/turn_angle.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integration");
  ros::NodeHandle n;

  VisibleFeatureManager visible_features;

  ros::Subscriber sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &VisibleFeatureManager::subscribeToLidar, &visible_features);
  ros::Subscriber sub_ball_harvest = n.subscribe<core_msgs::ball_position>(
      "/position", 1000, &VisibleFeatureManager::subscribeToCamera, &visible_features);
  // ros::Subscriber sub_line_tracing = n.subscribe<core_msgs::line_info>("/line_info", 1000, camera_Callback_2);
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

  std::vector<double> test_angles{
    0,
    angles::from_degrees(15),
    angles::from_degrees(30),
    angles::from_degrees(45),
    angles::from_degrees(60),
    angles::from_degrees(75),
    angles::from_degrees(90),
  };
  auto current_angle = test_angles.begin();
  int counter = 0;

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

    --counter;
    if (counter < 0)
    {
      counter = 500;
      task_executor.overrideTask(Task::TaskPtr(new TurnAngle(100000, *current_angle)));

      ++current_angle;
      if (current_angle == test_angles.end())
        current_angle = test_angles.begin();
    }

    // TODO 더 정확한 시간을 사용하기
    task_executor.runTaskInLoop(sleep_duration.toSec(), sleep_duration.toSec());

    sleep_duration.sleep();
    ros::spinOnce();
  }

  return 0;
}
