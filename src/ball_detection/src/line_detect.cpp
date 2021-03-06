#include <core_msgs/line_info.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

core_msgs::line_info msg1;
ros::Publisher dirPub;

cv::Mat Gauss(cv::Mat input)
{
  cv::Mat output;
  // Applying Gaussian Filter
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

void colorthresh(cv::Mat input)
{
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;
  msg1.image_width = s.width;
  msg1.image_height = s.height;
  auto c_x = 0.0;

  // Detect all objects within the HSV range
  //  cv::Mat img_hsv;
  //  cv::cvtColor(input, img_hsv, CV_BGR2HSV);
  cv::Scalar lower_black(0, 0, 0);
  cv::Scalar upper_black(30, 30, 30);
  cv::Mat img_mask;
  cv::inRange(input, lower_black, upper_black, img_mask);
  img_mask(cv::Rect(0, 0.2 * h, w, h - 0.2 * h)) = 0;
  // Find contours for better visualization
  cv::findContours(img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0)
  {
    auto area = 0;
    auto idx = 0;
    auto count = 0;
    while (count < v.size())
    {
      if (area < v[count].size())
      {
        idx = count;
        area = v[count].size();
      }
      count++;
    }
    cv::Rect rect = cv::boundingRect(v[idx]);
    cv::Point pt1, pt2, pt3;
    pt1.x = rect.x;
    pt1.y = rect.y;
    pt2.x = rect.x + rect.width;
    pt2.y = rect.y + rect.height;
    pt3.x = pt1.x + 5;
    pt3.y = pt1.y - 5;
    ROS_INFO("x = %d, y = %d, w = %d, h = %d", rect.x, rect.y, rect.width, rect.height);

    msg1.x = pt1.x;
    msg1.y = pt1.y;
    msg1.w = rect.width;
    msg1.h = rect.height;

    // Drawing the rectangle using points obtained
    // cv::rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
    // Inserting text box
    // cv::putText(input, "Line Detected", pt3, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }
  // Mask image to limit the future turns affecting the output
  img_mask(cv::Rect(0.7 * w, 0, 0.3 * w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3 * w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(img_mask);
  if (M.m00 > 0)
  {
    cv::Point p1(M.m10 / M.m00, M.m01 / M.m00);
    cv::circle(img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }
  c_x = M.m10 / M.m00;
  // Tolerance to choose directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask);
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  if (c_x < w / 2 - tol)
  {
    msg1.section = 0;
  }
  else if (c_x > w / 2 + tol)
  {
    msg1.section = 2;
  }
  else
  {
    msg1.section = 1;
  }
  // Search if no line detected
  if (count == 0)
  {
    msg1.section = 3;
  }
  // Output images viewed by the turtlebot
  // cv::namedWindow("Robot View", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  // cv::resizeWindow("Robot View", 640, 360);
  // cv::imshow("Robot View", input);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  static cv::Mat img, img_filt;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    // cv::waitKey(30);

    // Perform image processing
    img_filt = Gauss(img);
    colorthresh(img_filt);
    // Publish direction message
    dirPub.publish(msg1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  // Initializing node and object
  ros::init(argc, argv, "line_detect");
  ros::NodeHandle n;
  // Creating Publisher and subscriber
  ros::Subscriber sub = n.subscribe("/camera2/rgb/image_raw2", 1, imageCallback);

  dirPub = n.advertise<core_msgs::line_info>("/line_section", 1);

  ros::spin();
  // Closing image viewer
  // cv::destroyWindow("Robot View");
}
