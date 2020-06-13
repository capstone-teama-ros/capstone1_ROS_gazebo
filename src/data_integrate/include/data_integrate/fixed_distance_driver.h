#ifndef DATA_INTEGRATE_FIXED_DISTANCE_DRIVER_H
#define DATA_INTEGRATE_FIXED_DISTANCE_DRIVER_H

#include "./simple_wheel_controller.h"

/**
 * 입력한 거리만큼 직선으로 전진 또는 후진한 뒤 정지하는 운전자입니다.
 * 주의: 미끄러짐이나 장애물 등을 고려하지 않으므로 실제 이동한 거리는 입력한 거리와 다를 수 있습니다.
 */
class FixedDistanceDriver
{
public:
  /**
   * @param max_speed 전진 시 최대 속도 (m/s). 0보다 커야 합니다.
   * @param min_speed 후진 시 최대 속도 (m/s). 0보다 작아야 합니다.
   * @param distance_threshold 거리 계산 시 허용하는 오차 범위. 0 이상이어야 합니다.
   */
  FixedDistanceDriver(double max_speed, double min_speed, double distance_threshold);

  /**
   * 기존의 이동 명령을 취소하고, 이동해야 할 거리를 새로 입력합니다.
   * 0보다 작은 거리를 입력하면 후진합니다.
   *
   * @param distance  이동할 거리 (meters)
   */
  void setDistance(double distance);

  /**
   * 현재 운행 상태에 알맞게 바퀴 컨트롤러를 업데이트합니다.
   *
   * @param wheel_controller 바퀴 컨트롤러 객체
   * @param time_to_next_update 다음 updateController() 호출 전까지의 (예상) 시간 (seconds). 속도 계산에 필요합니다.
   */
  void updateController(SimpleWheelController& wheel_controller, double time_to_next_update);

private:
  double max_speed_ = 1;           ///< m/s
  double min_speed_ = -1;          ///< m/s
  double distance_threshold_ = 0;  ///< meters
  double distance_ = 0;            ///< meters
};

#endif  // DATA_INTEGRATE_FIXED_DISTANCE_DRIVER_H
