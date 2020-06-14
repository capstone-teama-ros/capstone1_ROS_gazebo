/**
 * @file StationaryAlignDriver를 테스트하기 위한 노드입니다.
 */

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <random>

#include "data_integrate/fixed_distance_driver.h"
#include "data_integrate/simple_wheel_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_fixed_distance_driver");
  ros::NodeHandle n("~");

  ros::Publisher left_wheel = n.advertise<std_msgs::Float64>("left_wheel", 10);
  ros::Publisher right_wheel = n.advertise<std_msgs::Float64>("right_wheel", 10);
  SimpleWheelController wheel_controller(left_wheel, right_wheel);

  ros::Duration publish_interval(0.025);  // seconds

  const double MAX_SPEED = 2.5;            // m/s
  const double MIN_SPEED = -0.75;          // m/s
  const double DISTANCE_THRESHOLD = 0.01;  // meters
  FixedDistanceDriver driver(MAX_SPEED, MIN_SPEED, DISTANCE_THRESHOLD);

  ROS_INFO("FixedDistanceDriver test node is publishing on:\n  %s\n  %s", left_wheel.getTopic().c_str(),
           right_wheel.getTopic().c_str());

  double target_distance = 0;
  double targetChangeTimer = 0;
  const double TARGET_CHANGE_INTERVAL = 5;  // seconds

  // 무작위 의사난수 생성용 코드
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<double> target_pos_distribution(-5, 10);

  while (ros::ok())
  {
    targetChangeTimer -= publish_interval.toSec();
    if (targetChangeTimer < 0)
    {
      target_distance = target_pos_distribution(rng);
      targetChangeTimer = TARGET_CHANGE_INTERVAL;
      ROS_INFO("Set new distance to %f (m)", target_distance);
      driver.setDistance(target_distance);
    }

    driver.updateController(wheel_controller, publish_interval.toSec());
    wheel_controller.publish();

    publish_interval.sleep();
  }

  return 0;
}
