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

  visualization_msgs::Marker marker;

  // Marker's frame (fixed frame)
  marker.header.frame_id = "/base_scan";
  marker.header.stamp = ros::Time(0);  // No timestamp; always display this

  // Unique identifier for the marker
  marker.ns = "my_shape";
  marker.id = 0;

  // Marker shape
  marker.type = visualization_msgs::Marker::CUBE;
  // Marker action
  marker.action = visualization_msgs::Marker::ADD;

  // Set marker initial pose
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;  // rotate about z-axis?
  marker.pose.orientation.w = 1.0;  // rotation amount?

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // Stay alive forever

  while (ros::ok())
  {
    auto& lidar_points = vfm.getLidarPoints();
    if (lidar_points.size() > 0)
    {
      // 첫 점의 좌표를 사용
      auto x = lidar_points.front().getRelX();
      auto y = lidar_points.front().getRelY();
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      ROS_INFO("Point highlighted: %f, %f", x, y);
    }

    marker_pub.publish(marker);

    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}