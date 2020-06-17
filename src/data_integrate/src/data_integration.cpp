#include <core_msgs/ball_position.h>
#include <core_msgs/line_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include "data_integrate/blackboard.h"
#include "data_integrate/direct_wheel_controller.h"
#include "data_integrate/features/visible_feature_manager.h"
#include "data_integrate/simple_wheel_controller.h"
#include "data_integrate/task_executor.h"
#include "data_integrate/tasks/goal_post_search_task.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integration");
  ros::NodeHandle n;

  VisibleFeatureManager visible_features;

  ros::Subscriber sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &VisibleFeatureManager::subscribeToLidar, &visible_features);
  ros::Subscriber sub_ball_harvest = n.subscribe<core_msgs::ball_position>(
      "/position", 1000, &VisibleFeatureManager::subscribeToCamera, &visible_features);
  ros::Subscriber lower_cam_sub = n.subscribe<core_msgs::ball_ch>(
      "/ball_ch", 1000, &VisibleFeatureManager::subscribeToLowerCamera, &visible_features);
  ros::Subscriber sub_line_tracing = n.subscribe<core_msgs::line_info>(
      "/line_info", 1000, &VisibleFeatureManager::subscribeToLineInfo, &visible_features);
  ros::Subscriber sub_imu =
      n.subscribe<sensor_msgs::Imu>("/imu", 1000, &VisibleFeatureManager::subscribeToImu, &visible_features);

  ros::Publisher fl_wheel = n.advertise<std_msgs::Float64>("/myrobot/FLwheel_velocity_controller/command", 10);
  ros::Publisher fr_wheel = n.advertise<std_msgs::Float64>("/myrobot/FRwheel_velocity_controller/command", 10);
  ros::Publisher bl_wheel = n.advertise<std_msgs::Float64>("/myrobot/BLwheel_velocity_controller/command", 10);
  ros::Publisher br_wheel = n.advertise<std_msgs::Float64>("/myrobot/BRwheel_velocity_controller/command", 10);

  ros::Publisher fl_publish = n.advertise<std_msgs::Float64>("myrobot/FLsuspension_position_controller/command", 10);
  ros::Publisher fr_publish = n.advertise<std_msgs::Float64>("/myrobot/FRsuspension_position_controller/command", 10);
  ros::Publisher bl_publish = n.advertise<std_msgs::Float64>("/myrobot/BLsuspension_position_controller/command", 10);
  ros::Publisher br_publish = n.advertise<std_msgs::Float64>("/myrobot/BRsuspension_position_controller/command", 10);

  SimpleWheelController wheel_controller(fl_wheel, fr_wheel, bl_wheel, br_wheel);
  DirectWheelController direct_controller(fl_wheel, fr_wheel, bl_wheel, br_wheel);
  Blackboard blackboard(visible_features, wheel_controller, direct_controller);
  TaskExecutor task_executor(blackboard);
  task_executor.overrideTask(Task::TaskPtr(new GoalPostSearchTask()));

  ros::Duration sleep_duration(0.025);

  while (ros::ok())
  {
    std_msgs::Float64 FL_position_msg;
    std_msgs::Float64 FR_position_msg;
    std_msgs::Float64 BL_position_msg;
    std_msgs::Float64 BR_position_msg;

    FL_position_msg.data = 0.0125;
    FR_position_msg.data = 0.0125;
    BL_position_msg.data = 0.0125;
    BR_position_msg.data = 0.0125;

    fl_publish.publish(FL_position_msg);
    fr_publish.publish(FR_position_msg);
    bl_publish.publish(BL_position_msg);
    br_publish.publish(BR_position_msg);

    // TODO 더 정확한 시간을 사용하기
    task_executor.runTaskInLoop(sleep_duration.toSec(), sleep_duration.toSec());

    sleep_duration.sleep();
    ros::spinOnce();
  }

  return 0;
}
