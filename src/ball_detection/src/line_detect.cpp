#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/line_info.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

core_msgs::line_info msg1;
Mat img;
Mat img_filt;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat Gauss(cv::Mat input) {
  cv::Mat output;
// Applying Gaussian Filter
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

void colorthresh(cv::Mat input) {
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;
  auto c_x = 0.0;
  // Detect all objects within the HSV range
//  Mat img_hsv;
//  cv::cvtColor(input, img_hsv, CV_BGR2HSV);
  Scalar LowerBlack;
  Scalar UpperBlack;
  LowerBlack = {0, 0, 0};
  UpperBlack = {30, 30, 30};
  Mat img_mask;
  cv::inRange(input, LowerBlack, UpperBlack, img_mask);
  img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0) {
  auto area = 0;
  auto idx = 0;
  auto count = 0;
  while (count < v.size()) {
    if (area < v[count].size()) {
       idx = count;
       area = v[count].size();
    }
    count++;
  }
  cv::Rect rect = boundingRect(v[idx]);
  cv::Point pt1, pt2, pt3;
  pt1.x = rect.x;
  pt1.y = rect.y;
  pt2.x = rect.x + rect.width;
  pt2.y = rect.y + rect.height;
  pt3.x = pt1.x+5;
  pt3.y = pt1.y-5;

  msg1.x = pt1.x;
  msg1.y = pt1.y;
  msg1.w = rect.width;
  msg1.h = rect.height;

  // Drawing the rectangle using points obtained
  rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
  // Inserting text box
  cv::putText(input, "Line Detected", pt3,
    CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }
  // Mask image to limit the future turns affecting the output
  img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(img_mask);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }
  c_x = M.m10/M.m00;
  // Tolerance to chooise directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask);
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  if (c_x < w/2-tol) {
    msg1.section = 0;
  } else if (c_x > w/2+tol) {
    msg1.section = 2;
  } else {
    msg1.section = 1;
  }
  // Search if no line detected
  if (count == 0) {
    msg1.section = 3;
  }
  // Output images viewed by the turtlebot
  cv::namedWindow("Turtlebot View");
  imshow("Turtlebot View", input);
}


int main(int argc, char **argv) {
    // Initializing node and object
    ros::init(argc, argv, "line_detect");
    ros::NodeHandle n;
    // Creating Publisher and subscriber
    ros::Subscriber sub = n.subscribe("/camera2/rgb/image_raw2",1,imageCallback);

    ros::Publisher dirPub = n.advertise<core_msgs::line_info>("/line_section", 1);

    while(ros::ok()){
        if (!img.empty()) {
            // Perform image processing
            img_filt = Gauss(img);
            colorthresh(img_filt);
            // Publish direction message
            dirPub.publish(msg1);
            }
        ros::spinOnce();
    }
    // Closing image viewer
    cv::destroyWindow("Turtlebot View");
}
