#include <angles/angles.h>
#include <core_msgs/ball_position.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <iterator>
#include <tuple>
#include <utility>

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

  RobotDrawing image(600, 600, 1 / 0.015);
  RobotDrawing abs_image(1200, 1200, 60);
  const double LIDAR_POINT_SIZE = image.toMeters(2);
  const int BALL_RADIUS = 5;
  const int ROBOT_SIZE = 10;

  // auto result1 = solveOrigin(RelPoint::fromRelXY(-2, 3), 3, RelPoint::fromRelXY(-1, 3 - std::sqrt(3)), 2);
  // ROS_INFO("%f, %f / %f, %f", result1.first.getRelX(), result1.first.getRelY(), result1.second.getRelX(),
  //          result1.second.getRelY());
  // auto result2 = solveOrigin(RelPoint::fromRelXY(-1, 4), 1.5, RelPoint::fromRelXY(-2, 3), 0.9);
  // ROS_INFO("%f, %f / %f, %f", result2.first.getRelX(), result2.first.getRelY(), result2.second.getRelX(),
  //          result2.second.getRelY());
  // auto result3 = solveOrigin(RelPoint::fromRelXY(0.9, 1), 0.6, RelPoint::fromRelXY(0.5, 0.3), 0.9);
  // ROS_INFO("%f, %f / %f, %f", result3.first.getRelX(), result3.first.getRelY(), result3.second.getRelX(),
  //          result3.second.getRelY());
  // ROS_ASSERT(0);

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

    // Draw absolute points
    abs_image.clear();
    abs_image.circle({ 4, 0.6 }, 3, Color::red(), CV_FILLED);
    for (auto& origin : vfm.getOrigins())
    {
      abs_image.drawMarker({ origin.getRelX(), origin.getRelY() }, Color::pink(), cv::MARKER_STAR, 6);
    }

    // wait for a key command. if 'q' is pressed, then program will be terminated.
    abs_image.draw("Absolute frame");
    if (image.drawAndWaitKey("Frame", 50) == 113)
    {
      break;
    }
    ros::spinOnce();
  }

  return 0;
}
