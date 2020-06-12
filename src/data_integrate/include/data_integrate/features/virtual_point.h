#ifndef DATA_INTEGRATE_FEATURES_VIRTUAL_POINT_H
#define DATA_INTEGRATE_FEATURES_VIRTUAL_POINT_H

#include "./past_feature.h"
#include "./rel_point.h"

/**
 * 로봇 좌표계 상의 가상의 점을 나타냅니다.
 * 복잡한 이동을 할 때 경유지로 사용할 수 있습니다.
 */
class VirtualPoint : public PastFeature
{
public:
  /**
   * 가상의 점을 생성합니다.
   *
   * @param distance 로봇으로부터 점까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 점의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   * @param reliability 점의 위치에 대한 신뢰도
   */
  VirtualPoint(double distance, double angle, Reliability reliability)
    : PastFeature(reliability), point_(distance, angle)
  {
  }

  /**
   * 가상의 점을 생성합니다. 신뢰도는 기본값을 사용합니다.
   *
   * @param distance 로봇으로부터 점까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 점의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   */
  VirtualPoint(double distance, double angle) : PastFeature(), point_(distance, angle)
  {
  }

  /**
   * 다른 가상의 점으로부터 가상의 점을 생성합니다.
   *
   * @param virtual_point 기존의 가상의 점
   * @param reliability 신뢰도
   */
  VirtualPoint(const VirtualPoint& virtual_point, Reliability reliability)
    : PastFeature(reliability), point_(virtual_point.point_)
  {
  }

  /**
   * 기존의 점으로부터 가상의 점을 생성합니다. 신뢰도는 기본값을 사용합니다.
   *
   * @param point 관측된 점
   */
  VirtualPoint(const RelPoint& point) : PastFeature(), point_(point)
  {
  }

  /**
   * 기존의 점으로부터 가상의 점을 생성합니다.
   *
   * @param point 관측된 점
   * @param reliability 점의 위치에 대한 신뢰도
   */
  VirtualPoint(const RelPoint& point, Reliability reliability) : PastFeature(reliability), point_(point)
  {
  }

  /**
   * 로봇 좌표계 상의 점의 x, y좌표를 바탕으로 가상의 점을 생성합니다. 신뢰도는 기본값을 사용합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 점의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 점의 y좌표 (meters)
   * @param reliability 점의 위치에 대한 신뢰도
   */
  static VirtualPoint fromRelXY(double rel_x, double rel_y, Reliability reliability)
  {
    return VirtualPoint(RelPoint::fromRelXY(rel_x, rel_y), reliability);
  }

  /**
   * 로봇 좌표계 상의 점의 x, y좌표를 바탕으로 가상의 점을 생성합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 점의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 점의 y좌표 (meters)
   */
  static VirtualPoint fromRelXY(double rel_x, double rel_y)
  {
    return VirtualPoint(RelPoint::fromRelXY(rel_x, rel_y));
  }

  /**
   * @returns 로봇 좌표계 위에서 점의 x좌표 (meters)
   */
  double getRelX() const
  {
    return point_.getRelX();
  }

  /**
   * @returns 로봇 좌표계 위에서 점의 y좌표 (meters)
   */
  double getRelY() const
  {
    return point_.getRelX();
  }

  /**
   * @returns 로봇으로부터 점까지의 거리 (meters)
   */
  double getDistance() const
  {
    return point_.getDistance();
  }

  /**
   * 로봇이 바라보는 방향(y축)과, 점의 위치 벡터 사이의 각을 구합니다. (radians)
   * 범위는 -π ~ π 입니다.
   * 양수일 경우 반시계 방향, 음수일 경우 시계 방향에 있습니다.
   */
  double getAngle() const
  {
    return point_.getAngle();
  }

  /**
   * 점의 좌표값을 RelPoint로 변환합니다.
   */
  const RelPoint& getPoint() const
  {
    return point_;
  }

private:
  RelPoint point_;
};

#endif  // DATA_INTEGRATE_FEATURES_VIRTUAL_POINT_H
