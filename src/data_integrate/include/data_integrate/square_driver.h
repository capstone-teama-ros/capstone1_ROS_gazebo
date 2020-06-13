#ifndef DATA_INTEGRATE_SQUARE_DRIVER_H
#define DATA_INTEGRATE_SQUARE_DRIVER_H

#include "./simple_wheel_controller.h"

/**
 * 현재 로봇이 실행하고 있는 운행 상태를 나타냅니다.
 */
enum class MoveActionState
{
  Stop,  ///< 정지한 상태
  Line,  ///< 정사각형의 변을 따라 직진한다.
  Turn,  ///< 정사각형의 꼭짓점에서 회전한다.
};

/**
 * 반시계 방향으로 정사각형을 그리는 운전 알고리즘. 정사각형을 모두 그리면 잠시 쉬었다가 다시 시작합니다.
 */
class SquareDriver
{
public:
  /**
   * @param side_length   한 변의 길이 (meters)
   * @param line_speed    한 변을 따라 이동할 때의 선속도 (m/s)
   * @param turn_speed    꼭짓점에서 회전할 때의 각속도 (rad/s)
   * @param rest_duration 정사각형을 1번 그린 후 쉬는 시간 (seconds)
   */
  SquareDriver(double side_length, double line_speed, double turn_speed, double rest_duration)
    : side_length_(side_length), line_speed_(line_speed), turn_speed_(turn_speed), rest_duration_(rest_duration)
  {
    // 비워놓음
  }

  /**
   * 현재 운행 상태를 시간이 @p time_passed 만큼 지나간 후의 상태로 변경합니다.
   * @param time_passed 지나간 시간
   */
  void updateState(double time_passed);

  /**
   * 현재 운행 상태에 알맞게 바퀴 컨트롤러를 업데이트합니다.
   */
  void updateController(SimpleWheelController& wheel_controller) const;

private:
  double side_length_ = 1;
  double line_speed_ = 1;
  double turn_speed_ = 1;
  double rest_duration_ = 1;
  double action_timer_ = 0;
  double line_segment_counter_ = 0;
  MoveActionState action_state_ = MoveActionState::Stop;
};

#endif  // DATA_INTEGRATE_SQUARE_DRIVER_H
