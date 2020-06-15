#include <ros/ros.h>
#include "data_integrate/robot_drawing.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_image_node");
  ros::NodeHandle n;

  RobotDrawing image(800, 600, 1 / 0.015 * 1.5);
  image.line({ 0, 0 }, { 1.0, 1.25 }, cv::viz::Color::white());
  image.circle({ 1, -1.5 }, 5, cv::viz::Color::blue(), CV_FILLED, cv::LINE_AA);
  image.rectangle({ -15, -15 }, { -1.15, -0.9 }, cv::viz::Color::green());
  image.drawMarker({ -2, 2 }, cv::viz::Color::yellow(), cv::MARKER_STAR, 10);

  while (ros::ok())
  {
    auto key = image.drawAndWaitKey("Some image", 50);
    if (key == 'q')
    {
      break;
    }
  }

  return 0;
}
