#include <core_msgs/ball_position.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "data_integrate/features/visible_feature_manager.h"
#include "data_integrate/robot_drawing.h"

int main(int argc, char** argv)
{
  using cv::viz::Color;

  ros::init(argc, argv, "data_show_node");
  ros::NodeHandle n("~");

  VisibleFeatureManager vfm;

  ros::Subscriber lidar_sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &VisibleFeatureManager::subscribeToLidar, &vfm);
  ros::Subscriber camera_sub =
      n.subscribe<core_msgs::ball_position>("/position", 1000, &VisibleFeatureManager::subscribeToCamera, &vfm);
  ros::Subscriber lower_cam_sub =
      n.subscribe<core_msgs::ball_ch>("/ball_ch", 1000, &VisibleFeatureManager::subscribeToLowerCamera, &vfm);

  RobotDrawing image(600, 600, 1 / 0.015);
  const double LIDAR_POINT_SIZE = image.toMeters(2);
  const int BALL_RADIUS = 5;
  const int ROBOT_SIZE = 10;

  while (ros::ok())
  {
    image.clear();

    // Drawing Lidar data
    for (auto& point : vfm.getLidarPoints())
    {
      auto x = point.getRelX();
      auto y = point.getRelY();
      image.line({ 0, 0 }, { x, y }, Color::teal(), 1, cv::LINE_AA);
      image.rectangle({ x - LIDAR_POINT_SIZE, y - LIDAR_POINT_SIZE }, { x + LIDAR_POINT_SIZE, y + LIDAR_POINT_SIZE },
                      Color::celestial_blue(), CV_FILLED);
    }

    // Drawing ball
    for (auto& ball : vfm.getBalls())
    {
      auto x = ball.getRelX();
      auto y = ball.getRelY();
      Color color;
      switch (ball.getColor())
      {
        case BallColor::Blue:
          color = Color::blue();
          break;
        case BallColor::Green:
          color = Color::green();
          break;
        case BallColor::Red:
          color = Color::red();
          break;
        default:
          ROS_ASSERT_MSG(0, "Unexpected color: %u", static_cast<unsigned int>(ball.getColor()));
      }
      image.circle({ x, y }, BALL_RADIUS, color, CV_FILLED, cv::LINE_AA);
    }

    // Drawing columns
    for (auto& column : vfm.getColumns())
    {
      image.drawMarker({ column.getRelX(), column.getRelY() }, Color::yellow(), cv::MARKER_STAR, 6);
    }

    // Drawing ROBOT
    image.drawMarker({ 0, 0 }, Color::yellow(), cv::MARKER_STAR, ROBOT_SIZE);

    // wait for a key command. if 'q' is pressed, then program will be terminated.
    if (image.drawAndWaitKey("Frame", 50) == 113)
    {
      break;
    }
    ros::spinOnce();
  }

  return 0;
}
