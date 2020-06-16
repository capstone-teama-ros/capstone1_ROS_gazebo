#ifndef DATA_INTEGRATE_SIMPLE_WHEEL_CONTROLLER_H
#define DATA_INTEGRATE_SIMPLE_WHEEL_CONTROLLER_H

#include <ros/ros.h>

/**
 * 아주 기초적인 동작만을 수행할 수 있는 운전 컨트롤러입니다.
 * 등속도 선운동, 등각속도 제자리 회전 운동, 정지의 3가지 동작만 수행 가능합니다.
 */
class SimpleWheelController
{
public:
  /**
   * @param lf_wheel 왼쪽 앞바퀴에 메시지를 전달할 publisher
   * @param rf_wheel 오른쪽 앞바퀴에 메시지를 전달할 publisher
   * @param lb_wheel 왼쪽 뒷바퀴에 메시지를 전달할 publisher
   * @param rb_wheel 오른쪽 뒷바퀴에 메시지를 전달할 publisher
   */
  SimpleWheelController(const ros::Publisher& lf_wheel, const ros::Publisher& rf_wheel, const ros::Publisher& lb_wheel,
                        const ros::Publisher& rb_wheel);

  /**
   * 일정한 속도로 선운동을 합니다. 회전은 하지 않습니다.
   * @p linear_speed 가 0보다 작으면 후진합니다.
   * 회전 중이었다면 회전을 중지합니다.
   *
   * @param linear_speed 선속도 (단위: m/s)
   */
  void moveLinear(double linear_speed);

  /**
   * @returns 현재 (목표로 한) 선속도 (m/s)
   */
  double getLinearSpeed() const
  {
    return linear_speed_;
  }

  /**
   * 제자리에서 일정한 각속도로 회전합니다. 등속도 운동 중이었다면 등속도 운동을 중지합니다.
   * 양수값을 주면 좌회전(반시계 방향), 음수값을 주면 우회전(시계 방향)합니다.
   * 선운동 중이였다면 선운동을 중지합니다.
   *
   * @param angular_speed 각속도 (단위: rad/s)
   */
  void turn(double angular_speed);

  /**
   * @returns 현재 (목표로 한) 각속도 (rad/s)
   */
  double getAngularSpeed() const
  {
    return angular_speed_;
  }

  /**
   * 선운동 또는 회전을 중지하고 제자리에 정지합니다.
   * 이미 정지한 경우 아무런 동작을 하지 않습니다.
   */
  void stop();

  /**
   * 각 바퀴의 publisher에게 현재 진행하고 있는 동작에 알맞는 메시지를 전달합니다.
   */
  void publish() const;

  /**
   * 로봇이 제자리에서 일정한 각속도로 회전하기 위해 각 바퀴에 전달해야 하는 속도 신호값을 계산합니다.
   *
   * @param robot_angular_velocity  로봇의 반시계 방향 회전 속도 (rad/s). 0보다 작으면 시계 방향의 회전 속도입니다.
   * @returns 우측 바퀴에 전달할 속도 신호값. 좌측 바퀴에는 이 값과 크기는 같고 부호가 반대인 신호값을 전달해야 합니다.
   */
  static double convertRobotAngularVelocityToWheelVelocity(double angular_velocity);

private:
  double linear_speed_ = 0;
  double angular_speed_ = 0;
  const ros::Publisher& lf_wheel_;
  const ros::Publisher& rf_wheel_;
  const ros::Publisher& lb_wheel_;
  const ros::Publisher& rb_wheel_;
};

#endif  // DATA_INTEGRATE_SIMPLE_WHEEL_CONTROLLER_H
