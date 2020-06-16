#include <core_msgs/ball_ch.h>
#include <core_msgs/ball_position.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>

// Setting Thresholds for red and blue part of image.
// Changable to fit your enviroment. If you want to use bgr, it should be different.

const cv::Scalar HSV_THRESHOLD_RED1_LOW(0, 134, 106);
const cv::Scalar HSV_THRESHOLD_RED1_HIGH(8, 255, 255);
const cv::Scalar HSV_THRESHOLD_RED2_LOW(169, 134, 106);
const cv::Scalar HSV_THRESHOLD_RED2_HIGH(180, 255, 255);
const cv::Scalar HSV_THRESHOLD_BLUE_LOW(100, 126, 60);
const cv::Scalar HSV_THRESHOLD_BLUE_HIGH(121, 255, 255);
const cv::Scalar HSV_THRESHOLD_GREEN_LOW(50, 126, 60);
const cv::Scalar HSV_THRESHOLD_GREEN_HIGH(70, 255, 255);

const int low_b_b = 150, high_g_b = 50, high_r_b = 50;

// Initialization of variable for camera calibration paramters.
// You should change this if you changed the size of the image.

float intrinsic_data[9] = { 1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0 };
float distortion_data[5] = { 0, 0, 0, 0, 0 };

// Minimum ball radius size by pixels. If ball is smaller than this, it won't be searched.
const int iMin_tracking_ball_size = 5;
// Initialization of variable for dimension of the target(real ball diameter by meter)
const float fball_diameter = 0.14;

// Setting cv::Mat variables for images.
cv::Mat buffer;
cv::Mat buffer2;
cv::Mat result;

// Setting Publishers
ros::Publisher pub;
ros::Publisher pub_markers;
ros::Publisher pub1;

// Declaring functions for image erode and dilaation.
void morphOps(cv::Mat& thresh)
{
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, erodeElement);
  cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, erodeElement);
}

void extractBall(cv::Mat& hsv_frame, int low_threshold, int ratio, int kernel_size, std::vector<cv::Vec4i>* hierarchy,
                 std::vector<std::vector<cv::Point> >* contours, std::vector<cv::Point2f>* centers,
                 std::vector<float>* radii)
{
  cv::Mat hsv_frame_1, hsv_frame_2;

  // Blur and erode, dilate
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(hsv_frame, hsv_frame_1, cv::MORPH_CLOSE, erodeElement);
  cv::morphologyEx(hsv_frame_1, hsv_frame_2, cv::MORPH_OPEN, erodeElement);
  cv::GaussianBlur(hsv_frame_2, hsv_frame, cv::Size(9, 9), 2, 2);

  // Canny Edge Detection
  cv::Mat img_canny;
  cv::Canny(hsv_frame, img_canny, low_threshold, low_threshold * ratio, kernel_size);

  // Finding Contours for blue threshold image
  hierarchy->clear();
  contours->clear();
  cv::findContours(img_canny, *contours, *hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Define variables for contour poly, center of circles, radius of circles
  std::vector<std::vector<cv::Point> > contours_poly(contours->size());
  centers->clear();
  centers->resize(contours->size());
  radii->clear();
  radii->resize(contours->size());

  // Finding balls by contours
  // Find polygon from contours and find the minimun size enclosing circle of that polygon.
  for (size_t i = 0; i < contours->size(); i++)
  {
    cv::approxPolyDP(contours->at(i), contours_poly[i], 1, true);
    cv::minEnclosingCircle(contours_poly[i], centers->at(i), radii->at(i));
  }
}
// Declaration of functions that calculates the ball position from pixel position.
std::vector<float> pixel2point(cv::Point center, int radius)
{
  std::vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;  //.x;// .at(0);
  y = center.y;  //.y;//
  u = (x - intrinsic_data[2]) / intrinsic_data[0];
  v = (y - intrinsic_data[5]) / intrinsic_data[4];
  Zc = (intrinsic_data[0] * fball_diameter) / (2 * (float)radius);
  Xc = u * Zc;
  Yc = v * Zc;
  Xc = std::roundf(Xc * 1000) / 1000;
  Yc = std::roundf(Yc * 1000) / 1000;
  Zc = std::roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

void addBallData(decltype(core_msgs::ball_position::blue_balls)* ball_data, const std::vector<cv::Vec4i>& hierarchy,
                 const std::vector<std::vector<cv::Point> >& contours, const std::vector<cv::Point2f>& centers,
                 const std::vector<float>& radii, const cv::Scalar& ball_color)
{
  for (size_t i = 0; i < contours.size(); i++)
  {
    if (hierarchy[i][3] == -1 && radii[i] > iMin_tracking_ball_size)
    {
      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px = centers[i].x;
      float py = centers[i].y;
      float pr = radii[i];

      // change the pixel value to real world value

      std::vector<float> ball_pos = pixel2point(centers[i], radii[i]);

      // draw the circle at the result cv::Mat matrix
      // putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos[0];
      float isy = ball_pos[1];
      float isz = ball_pos[2];

      std::string sx = std::to_string(isx);
      std::string sy = std::to_string(isy);
      std::string sz = std::to_string(isz);

      std::string text = "x: " + sx + ", y: " + sy + ", z: " + sz;
      cv::Scalar text_color = cv::Scalar(0, 255, 0);  // (blue, green, red)
      putText(result, text, centers[i], 2, 1, text_color, 2);
      circle(result, centers[i], static_cast<int>(radii[i]), ball_color, 2, 8, 0);

      // push back variables of real ball position to the message variable
      ball_data->emplace_back();
      auto new_ball = ball_data->back();
      new_ball.x = ball_pos[0];
      new_ball.y = ball_pos[1];
      new_ball.z = ball_pos[2];
    }
  }
}

void ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  cv::Mat intrinsic = cv::Mat(3, 3, CV_32F, intrinsic_data);
  cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, distortion_data);

  // Declare another cv::Mat variable to keep the image.
  cv::Mat frame;
  frame = buffer;

  cv::Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  cv::undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining cv::Mat variables for Threshold images.

  cv::Mat hsv_frame;
  cv::Mat hsv_frame_red;
  cv::Mat hsv_frame_red1;
  cv::Mat hsv_frame_red2;
  cv::Mat hsv_frame_blue;
  cv::Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result = calibrated_frame.clone();

  // Change RGB frame to HSV frame
  cv::cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  // Threshold
  cv::inRange(hsv_frame, HSV_THRESHOLD_RED1_LOW, HSV_THRESHOLD_RED1_HIGH, hsv_frame_red1);
  cv::inRange(hsv_frame, HSV_THRESHOLD_RED2_LOW, HSV_THRESHOLD_RED2_HIGH, hsv_frame_red2);
  cv::inRange(hsv_frame, HSV_THRESHOLD_BLUE_LOW, HSV_THRESHOLD_BLUE_HIGH, hsv_frame_blue);
  cv::inRange(hsv_frame, HSV_THRESHOLD_GREEN_LOW, HSV_THRESHOLD_GREEN_HIGH, hsv_frame_green);
  cv::addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

  // Canny Edge Detection
  int lowThreshold = 100;
  int ratio = 3;
  int kernel_size = 3;

  std::vector<cv::Vec4i> hierarchy_r, hierarchy_b, hierarchy_g;
  std::vector<std::vector<cv::Point> > contours_r, contours_b, contours_g;
  std::vector<cv::Point2f> center_r, center_b, center_g;
  std::vector<float> radius_r, radius_b, radius_g;

  extractBall(hsv_frame_red, lowThreshold, ratio, kernel_size, &hierarchy_r, &contours_r, &center_r, &radius_r);
  extractBall(hsv_frame_blue, lowThreshold, ratio, kernel_size, &hierarchy_b, &contours_b, &center_b, &radius_b);
  extractBall(hsv_frame_green, lowThreshold, ratio, kernel_size, &hierarchy_g, &contours_g, &center_g, &radius_g);

  // Declare message variable to publish
  core_msgs::ball_position msg;

  addBallData(&msg.blue_balls, hierarchy_b, contours_b, center_b, radius_b, cv::viz::Color::blue());
  addBallData(&msg.red_balls, hierarchy_r, contours_r, center_r, radius_r, cv::viz::Color::red());
  addBallData(&msg.green_balls, hierarchy_g, contours_g, center_g, radius_g, cv::viz::Color::green());

  // show what is published at the terminal
  ROS_INFO("blue: %lu", msg.blue_balls.size());
  for (auto& ball_pos : msg.blue_balls)
  {
    ROS_INFO("  x = %.4f, y = %.4f, z = %.4f", ball_pos.x, ball_pos.y, ball_pos.z);
  }
  ROS_INFO("red: %lu", msg.blue_balls.size());
  for (auto& ball_pos : msg.red_balls)
  {
    ROS_INFO("  x = %.4f, y = %.4f, z = %.4f", ball_pos.x, ball_pos.y, ball_pos.z);
  }
  ROS_INFO("green: %lu", msg.blue_balls.size());
  for (auto& ball_pos : msg.green_balls)
  {
    ROS_INFO("  x = %.4f, y = %.4f, z = %.4f", ball_pos.x, ball_pos.y, ball_pos.z);
  }

  pub.publish(msg);  // publish a message
  cv::imshow("result", result);
}

void ball_check()
{
  core_msgs::ball_ch msg;
  //  ROS_INFO("%s", buffer2.at<cv::Vec3b>(320,240)[0].c_str());          //확인해보려고 출력해보려했는데 오류떠서
  //  지워놨습니다.
  if (buffer2.at<cv::Vec3b>(320, 240)[0] > low_b_b && buffer2.at<cv::Vec3b>(320, 240)[1] < high_g_b &&
      buffer2.at<cv::Vec3b>(320, 240)[2] < high_r_b)
  {  //두번째 카메라에서 (320,240)의 b값이 최소값보다 큰지 확인한건데 원하는 위치에 맞게 조정 및 파란색만 잘 확인 될지
    msg.still_blue = 1;
  }
  else
  {
    msg.still_blue = 0;
  }
  pub1.publish(msg);  // publish a message
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->height == 480 && buffer.size().width == 320)
  {  // check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout << "resized" << std::endl;
    cv::resize(buffer, buffer, cv::Size(640, 480));
  }
  try
  {
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;
  }  // transfer the image data into buffer
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  ball_detect();
  cv::waitKey(1);
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->height == 480 && buffer.size().width == 320)
  {  // check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout << "resized" << std::endl;
    cv::resize(buffer2, buffer2, cv::Size(640, 480));
  }
  try
  {
    buffer2 = cv_bridge::toCvShare(msg, "bgr8")->image;
  }  // transfer the image data into buffer
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  ball_check();
  cv::waitKey(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detect_node");  // init ros nodd
  ros::NodeHandle nh;                         // create node handler
  image_transport::ImageTransport it(nh);     // create image transport and connect it to node hnalder
  image_transport::Subscriber sub1 = it.subscribe("/camera/rgb/image_raw", 1, imageCallback1);
  image_transport::Subscriber sub2 = it.subscribe("/camera2/rgb/image_raw2", 1, imageCallback2);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1);  // setting publisher
  pub1 = nh.advertise<core_msgs::ball_ch>("/ball_ch", 1);        // setting publisher
  ros::Rate loop_rate(30);
  cv::namedWindow("result", cv::WINDOW_NORMAL);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
