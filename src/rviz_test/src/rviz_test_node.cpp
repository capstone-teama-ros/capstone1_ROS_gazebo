/**
 * @file RViz로 LIDAR 데이터를 시각화하는 것을 테스트하는 노드입니다.
 */

#include <core_msgs/ball_position.h>
#include <features/visible_feature_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

class RvizDrawer
{
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_test");
  ros::NodeHandle n("~");

  VisibleFeatureManager vfm;

  ros::Subscriber lidar_sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &VisibleFeatureManager::subscribeToLidar, &vfm);
  ros::Subscriber camera_sub =
      n.subscribe<core_msgs::ball_position>("/position", 10, &VisibleFeatureManager::subscribeToCamera, &vfm);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  ROS_INFO("\"%s\" started", ros::this_node::getName().c_str());
  ros::Rate spin_rate(10);  // Hz

  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
