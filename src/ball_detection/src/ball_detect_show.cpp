/**
 * @file ball_detect.cpp의 디버그용 버전입니다. 이미지 분석 결과를 별도의 창에 띄워 표시합니다.
 */

#include <core_msgs/ball_ch.h>
#include <core_msgs/ball_position.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <sstream>

#include "ball_detection/ball_detect.h"

// Setting Publishers
ros::Publisher pub;
ros::Publisher pub1;

/**
 * @param result  그림을 그릴 이미지
 * @param data    데이터
 * @param ball_color 공 주위에 그릴 원의 색깔
 */
void drawBallData(cv::Mat* result, const BallDetectData& data, const cv::Scalar& ball_color)
{
  const cv::Scalar TEXT_COLOR = cv::Scalar(0, 255, 0);  // (blue, green, red)

  for (size_t i = 0; i < data.contours.size(); i++)
  {
    if (data.hierarchy[i][3] == -1 && data.radii[i] > iMin_tracking_ball_size)
    {
      float px = data.centers[i].x;
      float py = data.centers[i].y;
      float pr = data.radii[i];
      auto ball_pos = pixel2point(data.centers[i], data.radii[i]);

      std::ostringstream oss;
      oss << "x: " << ball_pos.x << ", y: " << ball_pos.y << ", z: " << ball_pos.z;
      cv::putText(*result, oss.str(), data.centers[i], 2, 1, TEXT_COLOR, 2);
      cv::circle(*result, data.centers[i], static_cast<int>(data.radii[i]), ball_color, 2, 8, 0);
    }
  }
}

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
  cv::Mat result = calibrated_frame.clone();  // 결과물 그리는 용도

  core_msgs::ball_position cam1_msg;
  auto data = ball_detect(calibrated_frame, &cam1_msg);

  drawBallData(&result, data.red_data, cv::viz::Color::red());
  drawBallData(&result, data.blue_data, cv::viz::Color::blue());
  drawBallData(&result, data.green_data, cv::viz::Color::green());
  cv::imshow("result", result);

  pub.publish(cam1_msg);
  cv::waitKey(1);
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
  ros::init(argc, argv, "ball_detect_show_node");  // init ros node
  ros::NodeHandle nh;                              // create node handle
  image_transport::ImageTransport it(nh);          // create image transport and connect it to node handle
  image_transport::Subscriber sub1 = it.subscribe("/camera/rgb/image_raw", 1, imageCallback1);
  image_transport::Subscriber sub2 = it.subscribe("/camera2/rgb/image_raw2", 1, imageCallback2);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1);  // setting publisher
  pub1 = nh.advertise<core_msgs::ball_ch>("/ball_ch", 1);        // setting publisher

  cv::namedWindow("result", cv::WINDOW_NORMAL);
  ros::spin();

  return 0;
}
