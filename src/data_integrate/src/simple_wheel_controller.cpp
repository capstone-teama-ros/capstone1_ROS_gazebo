#include "data_integrate/simple_wheel_controller.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

SimpleWheelController::SimpleWheelController(const ros::Publisher& left_wheel, const ros::Publisher& right_wheel)
  : left_wheel_(left_wheel), right_wheel_(right_wheel)
{
}

void SimpleWheelController::moveLinear(double linear_speed)
{
  stop();
  linear_speed_ = linear_speed;
}

void SimpleWheelController::turn(double angular_speed)
{
  stop();
  angular_speed_ = angular_speed;
}

void SimpleWheelController::stop()
{
  linear_speed_ = 0;
  angular_speed_ = 0;
}

void SimpleWheelController::publish() const
{
  std_msgs::Float64 left_wheel_msg;
  std_msgs::Float64 right_wheel_msg;

  /// 실험적으로 구한 비례상수
  const double LINEAR_SPEED_FACTOR = 1 / 0.054;
  auto wheel_angular_speed = convertRobotAngularVelocityToWheelVelocity(angular_speed_);

  left_wheel_msg.data = LINEAR_SPEED_FACTOR * linear_speed_ - wheel_angular_speed;
  right_wheel_msg.data = LINEAR_SPEED_FACTOR * linear_speed_ + wheel_angular_speed;

  left_wheel_.publish(left_wheel_msg);
  right_wheel_.publish(right_wheel_msg);
}

using InterPoint = std::pair<double, double>;

/**
 * 선형보간법에 사용할 기준점의 목록입니다.
 * 각 항목은 { 로봇의 각속도, 바퀴의 각속도 }의 쌍이며, 실험적으로 구한 것입니다.
 *
 * 주의사항:
 *  - 이 배열은 크기순으로 정렬되어 있어야 합니다.
 *  - 첫 값은 항상 (0, 0)이어야 합니다.
 *  - 중복되는 값이 없어야 합니다.
 */
const InterPoint ROBOT_TO_WHEEL_ANGULAR[] = {
  { 0, 0 },
  { 0.31573, 1.30900 },
  { 0.80468, 2.61799 },
  { 1.29654, 3.92699 },
  { 1.80750, 5.23599 },
  { 2.31900, 6.54499 },
  { 2.57867, 7.85398 },
};

double SimpleWheelController::convertRobotAngularVelocityToWheelVelocity(double robot_angular_velocity)
{
  auto robot_angular_speed = std::abs(robot_angular_velocity);

  // 이진 탐색(binary search)으로 목표한 속도값의 양쪽에 있는 기준점 a, b를 찾는다
  auto r2w_begin = std::begin(ROBOT_TO_WHEEL_ANGULAR);
  auto r2w_end = std::end(ROBOT_TO_WHEEL_ANGULAR);
  InterPoint search_target(robot_angular_speed, 0);
  auto b = std::lower_bound(r2w_begin, r2w_end, search_target,
                            [=](const InterPoint& p, const InterPoint& q) { return p.first < q.first; });

  if (r2w_begin == b)
  {
    // 속력이 0인 경우
    return 0;
  }
  ROS_ASSERT_MSG(b < r2w_end, "Robot angular speed is too large to convert: %f", robot_angular_velocity);
  auto a = b - 1;

  // 선형 보간(linear interpolation)으로 바퀴의 각속력을 구한다
  auto a_robot = a->first, a_wheel = a->second;
  auto b_robot = b->first, b_wheel = b->second;
  ROS_ASSERT_MSG(a_wheel != b_wheel, "Duplicate entry in ROBOT_TO_WHEEL_ANGULAR: %f", a_wheel);
  auto wheel_speed = (robot_angular_speed - a_robot) / (b_robot - a_robot) * (b_wheel - a_wheel) + a_wheel;

  if (robot_angular_velocity >= 0)
  {
    return wheel_speed;
  }
  else
  {
    return -wheel_speed;
  }
}
