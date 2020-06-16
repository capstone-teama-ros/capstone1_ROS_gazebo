/**
 * @file 이미지 분석 결과를 메시지로 publish합니다. 따로 창을 띄워 표시하지 않습니다.
 */

#include <core_msgs/ball_ch.h>
#include <core_msgs/ball_position.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <sstream>

#include "ball_detection/ball_detect.h"

// Setting Publishers
ros::Publisher pub;
ros::Publisher pub1;

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  static cv::Mat buffer;
  try
  {
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;
  }  // transfer the image data into buffer
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  cv::Mat calibrated_frame = calibrateFrame(buffer);

  core_msgs::ball_position cam1_msg;
  ball_detect(calibrated_frame, &cam1_msg);

  pub.publish(cam1_msg);
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat buffer;
  try
  {
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;
  }  // transfer the image data into buffer
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  core_msgs::ball_ch cam2_msg;
  cam2_msg.still_blue = hasBlueBallInCamera2(buffer);
  pub1.publish(cam2_msg);  // publish a message
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detect_node");  // init ros node
  ros::NodeHandle nh;                         // create node handle
  image_transport::ImageTransport it(nh);     // create image transport and connect it to node handle
  image_transport::Subscriber sub1 = it.subscribe("/camera/rgb/image_raw", 1, imageCallback1);
  image_transport::Subscriber sub2 = it.subscribe("/camera2/rgb/image_raw2", 1, imageCallback2);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1);  // setting publisher
  pub1 = nh.advertise<core_msgs::ball_ch>("/ball_ch", 1);        // setting publisher

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
