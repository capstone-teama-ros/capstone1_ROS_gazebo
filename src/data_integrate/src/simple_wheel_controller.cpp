#include <std_msgs/Float64.h>
#include "data_integrate/simple_wheel_controller.h"

SimpleWheelController::SimpleWheelController(const ros::Publisher &left_wheel, const ros::Publisher &right_wheel)
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

  // TODO: 올바른 계수를 직접 계산하거나 실험적으로 구해야 함
  const double LINEAR_SPEED_FACTOR = 2.5;
  const double ANGULAR_SPEED_FACTOR = 2.7;  // 실제 값과 매우 가까운 듯? 30 deg/s 로 움직였을 때 상당히 근접했음

  left_wheel_msg.data = LINEAR_SPEED_FACTOR * linear_speed_ - ANGULAR_SPEED_FACTOR * angular_speed_;
  right_wheel_msg.data = LINEAR_SPEED_FACTOR * linear_speed_ + ANGULAR_SPEED_FACTOR * angular_speed_;

  left_wheel_.publish(left_wheel_msg);
  right_wheel_.publish(right_wheel_msg);
}
