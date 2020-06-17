#include "data_integrate/direct_wheel_controller.h"

#include <std_msgs/Float64.h>

void DirectWheelController::publish() const
{
  std_msgs::Float64 left_wheel_msg;
  std_msgs::Float64 right_wheel_msg;

  left_wheel_msg.data = left_wheel_speed_;
  right_wheel_msg.data = right_wheel_speed_;

  lf_wheel_.publish(left_wheel_msg);
  rf_wheel_.publish(right_wheel_msg);
  lb_wheel_.publish(left_wheel_msg);
  rb_wheel_.publish(right_wheel_msg);
}
