/**
 * @file VisibleFeatureManager를 테스트하기 위한 노드입니다.
 */

#include <ros/ros.h>
#include "data_integrate/features/visible_feature_manager.h"

/**
 * 테스트할 용도로 상속함
 */
class TestVisibleFeatureManager : public VisibleFeatureManager
{
public:
  void subscribeToCamera(const core_msgs::ball_position::ConstPtr& msg);
  void subscribeToLidar(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void TestVisibleFeatureManager::subscribeToLidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  const auto before = ros::WallTime::now();
  VisibleFeatureManager::subscribeToLidar(msg);
  const auto after = ros::WallTime::now();

  size_t num_points = getLidarPoints().size();
  auto processing_msec = (after.toSec() - before.toSec()) * 1000;
  ROS_INFO("Received %lu points from LIDAR (processing took %f msec)", num_points, processing_msec);
}

void TestVisibleFeatureManager::subscribeToCamera(const core_msgs::ball_position::ConstPtr& msg)
{
  const auto before = ros::WallTime::now();
  VisibleFeatureManager::subscribeToCamera(msg);
  const auto after = ros::WallTime::now();

  int blue_balls = 0;
  int red_balls = 0;
  int green_balls = 0;
  for (auto& ball : getBalls())
  {
    auto color = ball.getColor();
    switch (color)
    {
      case BallColor::Blue:
        ++blue_balls;
        break;
      case BallColor::Red:
        ++red_balls;
        break;
      case BallColor::Green:
        ++green_balls;
        break;
      default:
        ROS_ASSERT_MSG(0, "Unexpected color: %u", static_cast<unsigned int>(color));
    }
  }

  auto processing_msec = (after.toSec() - before.toSec()) * 1000;
  ROS_INFO("Received balls from camera (processing took %f msec):\n  %u blue, %u red, %u green", processing_msec,
           blue_balls, red_balls, green_balls);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_visible_feature_manager");
  ros::NodeHandle n("~");

  TestVisibleFeatureManager test_visible_feature_manager;

  ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &TestVisibleFeatureManager::subscribeToLidar, &test_visible_feature_manager);
  ros::Subscriber camera_sub = n.subscribe<core_msgs::ball_position>(
      "/position", 1000, &TestVisibleFeatureManager::subscribeToCamera, &test_visible_feature_manager);

  ROS_INFO("%s has subscribed to %s, %s", ros::this_node::getName().c_str(), lidar_sub.getTopic().c_str(),
           camera_sub.getTopic().c_str());

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
