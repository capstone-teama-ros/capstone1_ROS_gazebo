#ifndef BALL_DETECTION_BALL_DETECT_H
#define BALL_DETECTION_BALL_DETECT_H

#include <core_msgs/ball_ch.h>
#include <core_msgs/ball_position.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <vector>

// Minimum ball radius size by pixels. If ball is smaller than this, it won't be searched.
const int iMin_tracking_ball_size = 5;
// Initialization of variable for dimension of the target(real ball diameter by meter)
const float fball_diameter = 0.14;

/**
 * 한 종류의 공을 추출하는 데 필요한 데이터
 */
struct BallDetectData
{
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Point2f> centers;
  std::vector<float> radii;
};

/**
 * 한 이미지에서 여러 종류의 공을 추출하는 데 필요한 데이터
 */
struct AllBallData
{
  BallDetectData red_data;
  BallDetectData blue_data;
  BallDetectData green_data;
};

/**
 * 공 탐지를 위한 전처리 작업
 * 이 함수가 생성한 이미지을 @c ball_detect() 에 넣어주면 됩니다.
 * 또한 생성한 이미지를 시각화하는 용도로 쓸 수 있습니다.
 */
cv::Mat calibrateFrame(const cv::Mat& buffer);

/**
 * 공 탐지 알고리즘. 직접 메시지를 publish 하지 않으니까 호출하는 코드가 따로 해줘야 합니다.
 *
 * @param rgb_frame 분석할 이미지. @c calibrateFrame() 으로 전처리된 이미지여야 합니다.
 * @param msg Publish할 데이터를 담을 구조체
 * @returns 분석 완료된 데이터
 */
AllBallData ball_detect(const cv::Mat& rgb_frame, core_msgs::ball_position* msg);

/**
 * 2번 카메라로 로봇 밑에 파란 공이 있는지 탐지하는 함수입니다.
 * 직접 메시지를 publish 하지 않으니까 호출하는 코드가 따로 해줘야 합니다.
 */
bool hasBlueBallInCamera2(const cv::Mat& buffer);

// Declaration of functions that calculates the ball position from pixel position.
cv::Point3f pixel2point(cv::Point center, int radius);

#endif  // BALL_DETECTION_BALL_DETECT_H
