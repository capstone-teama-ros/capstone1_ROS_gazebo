#ifndef DATA_INTEGRATE_STATIONARY_ALIGN_DRIVER_H
#define DATA_INTEGRATE_STATIONARY_ALIGN_DRIVER_H

#include "./simple_wheel_controller.h"

/**
 * 자신에 대한 상대 위치 (x, y)에 있는 점이 정면 방향에 오도록 제자리에서 회전하는 운전자입니다.
 * 자기 자신은 항상 원점(0, 0)에 있으며, y축 방향을 바라보고 있다고 가정합니다.
 */
class StationaryAlignDriver
{
public:
  /**
   * @param max_turn_speed      최대 회전 속도 (rad/s)
   * @param angle_threshold     "정면"으로 취급하는 오차 범위 각 (radians)
   * @param distance_threshold  가까운 점을 구분하는 최대 거리 (meters)
   */
  StationaryAlignDriver(double max_turn_speed, double angle_threshold, double distance_threshold);

  /**
   * 추적하는 점의 위치를 업데이트합니다. 각 좌표는 운전자 자신을 원점으로 하고, 운전자가 바라보는 방향을 y축 방향으로
   * 하는 좌표계 상의 좌표여야 합니다.
   *
   * @param rel_x x좌표 (meters)
   * @param rel_y y좌표 (meters)
   */
  void updatePoint(double rel_x, double rel_y);

  /**
   * 현재 운행 상태에 알맞게 바퀴 컨트롤러를 업데이트합니다.
   *
   * @param wheel_controller 바퀴 컨트롤러 객체
   * @param time_to_next_update 다음 updateController() 호출 전까지의 (예상) 시간 (seconds). 속도 계산에 필요합니다.
   */
  void updateController(SimpleWheelController& wheel_controller, double time_to_next_update) const;

private:
  double rel_x_ = 0;
  double rel_y_ = 0;
  double max_turn_speed_ = 10;     ///< rad/s
  double angle_threshold_ = 0;     ///< radians
  double distance_threshold_ = 0;  ///< meters
};

#endif  // DATA_INTEGRATE_STATIONARY_ALIGN_DRIVER_H
