#ifndef DATA_INTEGRATE_DIRECT_WHEEL_CONTROLLER_H
#define DATA_INTEGRATE_DIRECT_WHEEL_CONTROLLER_H

#include <ros/ros.h>

/**
 * 양쪽 바퀴에 보낼 속도 신호값을 직접 지정할 수 있는 컨트롤러입니다.
 */
class DirectWheelController
{
public:
  /**
   * @param lf_wheel 왼쪽 앞바퀴에 메시지를 전달할 publisher
   * @param rf_wheel 오른쪽 앞바퀴에 메시지를 전달할 publisher
   * @param lb_wheel 왼쪽 뒷바퀴에 메시지를 전달할 publisher
   * @param rb_wheel 오른쪽 뒷바퀴에 메시지를 전달할 publisher
   */
  DirectWheelController(const ros::Publisher& lf_wheel, const ros::Publisher& rf_wheel, const ros::Publisher& lb_wheel,
                        const ros::Publisher& rb_wheel)
    : lf_wheel_(lf_wheel), lb_wheel_(lb_wheel), rf_wheel_(rf_wheel), rb_wheel_(rb_wheel)
  {
  }

  /**
   * 양쪽 바퀴들의 속도를 직접 지정합니다.
   *
   * @param left_speed  왼쪽 바퀴에 보낼 신호값
   * @param right_speed 오른쪽 바퀴에 보낼 신호값
   */
  void setWheelSpeeds(double left_speed, double right_speed)
  {
    left_wheel_speed_ = left_speed;
    right_wheel_speed_ = right_speed;
  }

  void setLeftWheelSpeed(double left_speed)
  {
    left_wheel_speed_ = left_speed;
  }

  void setRightWheelSpeed(double right_speed)
  {
    right_wheel_speed_ = right_speed;
  }

  double getLeftWheelSpeed() const
  {
    return left_wheel_speed_;
  }

  double getRightWheelSpeed() const
  {
    return right_wheel_speed_;
  }

  /**
   * 모든 운동을 중지하고 제자리에 정지합니다.
   * 이미 정지한 경우 아무런 동작을 하지 않습니다.
   */
  void stop()
  {
    left_wheel_speed_ = 0;
    right_wheel_speed_ = 0;
  }

  /**
   * 각 바퀴의 publisher에게 현재 진행하고 있는 동작에 알맞는 메시지를 전달합니다.
   */
  void publish() const;

private:
  double left_wheel_speed_ = 0;
  double right_wheel_speed_ = 0;
  const ros::Publisher& lf_wheel_;
  const ros::Publisher& rf_wheel_;
  const ros::Publisher& lb_wheel_;
  const ros::Publisher& rb_wheel_;
};

#endif  // DATA_INTEGRATE_DIRECT_WHEEL_CONTROLLER_H
