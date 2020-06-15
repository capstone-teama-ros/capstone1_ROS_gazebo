#include "data_integrate/robot_drawing.h"

#include <ros/ros.h>
#include <opencv2/highgui.hpp>

void RobotDrawing::line(cv::Point2d pt1, cv::Point2d pt2, const cv::Scalar& color, int thickness, int lineType)
{
  cv::line(image_, fromRobotFrame(pt1), fromRobotFrame(pt2), color, thickness, lineType);
}

void RobotDrawing::circle(cv::Point2d center, int radius, const cv::Scalar& color, int thickness, int lineType)
{
  cv::circle(image_, fromRobotFrame(center), radius, color, thickness, lineType);
}

void RobotDrawing::drawMarker(cv::Point2d position, const cv::Scalar& color, int markerType, int markerSize,
                              int thickness, int line_type)
{
  cv::drawMarker(image_, fromRobotFrame(position), color, markerType, markerSize, thickness, line_type);
}

void RobotDrawing::rectangle(cv::Point2d pt1, cv::Point2d pt2, const cv::Scalar& color, int thickness, int lineType)
{
  cv::rectangle(image_, fromRobotFrame(pt1), fromRobotFrame(pt2), color, thickness, lineType);
}

int RobotDrawing::drawAndWaitKey(const cv::String& winname, int delay) const
{
  cv::imshow(winname, image_);
  int key = cv::waitKey(delay);
  ROS_DEBUG("%s received key code: %d", winname.c_str(), key);
  return key;
}
