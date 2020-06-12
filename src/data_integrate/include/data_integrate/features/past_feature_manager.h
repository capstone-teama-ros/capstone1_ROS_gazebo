#ifndef DATA_INTEGRATE_FEATURES_PAST_FEATURE_MANAGER_H
#define DATA_INTEGRATE_FEATURES_PAST_FEATURE_MANAGER_H

#include <unordered_map>

#include "./past_ball.h"
#include "./virtual_point.h"

/**
 * 과거에 보았던 중요한 지형지물의 정보를 관리하는 클래스입니다.
 * 과거에 봤던 지형지물에 고유한 ID를 붙여서 관리합니다.
 * 각 지형지물의 신뢰성은 지나간 시간에 대한 지수함수의 형태로 감소합니다.
 */
class PastFeatureManager
{
public:
  /**
   * @param decay_rate 1초당 신뢰성이 감소하는 비율. 0보다 크고 1보다 작아야 합니다.
   */
  PastFeatureManager(double decay_rate);

  // 공과 같은 중요한 지형지물을 식별하기 위해 부여하는 고유한 값
  using FeatureId = unsigned int;
  using BallCollection = std::unordered_map<FeatureId, PastBall>;
  using VirtualPointCollection = std::unordered_map<FeatureId, VirtualPoint>;

  /**
   * 과거에 관측했던 공들의 목록을 가져옵니다.
   */
  const BallCollection& getBalls() const;

  /**
   * 새로운 공을 추가합니다.
   *
   * @param ball 기억할 공의 정보
   * @returns 새로운 공에 할당한 ID값
   */
  FeatureId addBall(const PastBall& ball);

  /**
   * 과거에 본 공을 ID 값으로 찾습니다. ID가 일치하는 공이 없으면 @c nullptr 를 돌려줍니다.
   *
   * @param id 공의 ID
   * @returns 일치하는 공을 가리키는 포인터 또는 @c nullptr
   */
  const PastBall* getBall(FeatureId id) const;

  /**
   * 추적 중인 가상의 점의 목록을 가져옵니다.
   */
  const VirtualPointCollection& getVirtualPoints() const
  {
    return virtual_points_;
  }

  /**
   * 새로운 가상의 점을 추가합니다.
   *
   * @param virtual_point 기억할 가상의 점의 정보
   * @returns 가상의 점에 할당한 ID값
   */
  FeatureId addVirtualPoint(const VirtualPoint& virtual_point);

  /**
   * 과거에 본 가상의 점을 ID 값으로 찾습니다. ID가 일치하는 점이 없으면 @c nullptr 를 돌려줍니다.
   *
   * @param id 가상점의 ID
   * @returns 일치하는 가상의 점을 가리키는 포인터 또는 @c nullptr
   */
  const VirtualPoint* getVirtualPoint(FeatureId id) const;

  /**
   * 기억 중인 모든 feature 정보를 지웁니다.
   */
  void clearAllFeatures();

  /**
   * 모든 지형지물의 신뢰성을 @p time_passed 초만큼 시간이 지난 후의 값으로 변경합니다.
   *
   * @param time_passed 지나간 시간 (seconds)
   */
  void updateReliability(double time_passed);

  /**
   * 모든 지형지물의 위치를 로봇 좌표계 기준으로 (x, y)만큼 평행이동합니다.
   * (로봇 자신이 (-x, -y)만큼 이동한 것과 같습니다.)
   *
   * @param x x축 방향으로 평행이동할 거리 (meters)
   * @param y y축 방향으로 평행이동할 거리 (meters)
   */
  void translateObjectsXY(double x, double y);

  /**
   * 로봇 자신이 (x, y)만큼 이동한 것으로 계산하여, 모든 지형지물의 위치를 (-x, -y)만큼 평행이동합니다.
   *
   * @param x x축 방향으로 평행이동할 거리 (meters)
   * @param y y축 방향으로 평행이동할 거리 (meters)
   */
  void translateSelfXY(double x, double y);

  /**
   * 로봇을 기준으로 하여, 모든 지형지물의 위치를 반시계 방향으로 @p angle 만큼 회전시킵니다.
   * (로봇 자신이 @c -angle 만큼 반시계 방향으로 회전한 것과 같습니다.)
   *
   * @param angle 회전각 (radians)
   */
  void rotateObjectsZ(double angle);

  /**
   * 로봇 자신이 반시계 방향으로 @p angle 만큼 회전한 것으로 계산하여, 모든 지형지물의 위치를 @c -angle 만큼 반시계
   * 방향으로 회전시킵니다.
   *
   * @param angle 회전각 (radians)
   */
  void rotateSelfZ(double angle);

  /**
   * 새로운 지형지물을 만들기 위한 ID 값을 생성합니다.
   * 기존의 지형지물이 사용하는 ID값과 겹칠 수 있으니 @c isFeatureIdInUse() 로 검사해야 합니다.
   */
  static FeatureId generateId();

  /**
   * 입력한 feature ID를 이 feature manager가 이미 사용 중인지 확인합니다.
   */
  bool isFeatureIdInUse(FeatureId id) const;

private:
  double decay_rate_;
  BallCollection balls_;
  VirtualPointCollection virtual_points_;
};

#endif  // DATA_INTEGRATE_FEATURES_PAST_FEATURE_MANAGER_H
