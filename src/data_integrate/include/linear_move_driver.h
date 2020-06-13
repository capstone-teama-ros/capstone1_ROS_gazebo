#ifndef DATA_INTEGRATE_LINEAR_MOVE_DRIVER_H
#define DATA_INTEGRATE_LINEAR_MOVE_DRIVER_H

#include "../include/simple_wheel_controller.h"

/*
이 함수가 뭘 할까요
- 목표 지점과 align이 되어 있는지 확인하고 가능하다면 움직이기.
- 우선 장애물은 생각하지 말고 바로 움직이면 공을 주울 수 있다는 가정 하에
- 거리가 충분히 가까우면 멈춰서 다시 align하도록 나중에 연결 필요
*/

class LinearMoveDriver
{
public:
	/**
	 * @param linear_speed : 움직이는 선형 속도 (m/s)
	 * @param align_threshold : 직진하면서 공과 로봇이 align되었다고 판단하는 각도 (degrees)
	 * @param distance_threshold : 공과 매우 가까워졌다고 판단 하는 거리 (meters)
	 */
	LinearMoveDriver(double linear_speed, double align_threshold, double distance_threshold);

	/**
	 * 추적하는 공의 위치를 업데이트한다.
	 */
	void updatePoint(double rel_x, double rel_y);

	/**
	 * 현재 상태에 알맞게 직진할지 멈추고 다른 동작을 수행할지 판단
	 * @param wheel_controller 바퀴 컨트롤러 객체
	 * 속도 계산은 여기서 따로 안하고 publish할 linear_speed를 지정해줄 것임
	 */
	void LinearMoveDriver::updateController(SimpleWheelController& wheel_controller) const;

private:
	double linear_speed_ = 0.5;					// m/s
	double align_threshold_ = 3;				// degrees
	double distance_threshold_ = 0.05;	// meters
};

#endif	// DATA_INTEGRATE_LINEAR_MOVE_DRIVER_H
