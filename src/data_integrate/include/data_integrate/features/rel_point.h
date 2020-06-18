#ifndef DATA_INTEGRATE_FEATURES_REL_POINT_H
#define DATA_INTEGRATE_FEATURES_REL_POINT_H

#include "./feature.h"

/**
 * 로봇 좌표계 상의 점 하나의 위치를 나타냅니다.
 */
class RelPoint : public Feature
{
public:
  /**
   * @param distance 로봇으로부터 점까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 점의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   */
  RelPoint(double distance, double angle) : Feature(distance, angle)
  {
  }

  /**
   * @param feature Feature to copy position data from.
   */
  RelPoint(const Feature& feature) : Feature(feature)
  {
  }

  /**
   * 로봇 좌표계 상의 점의 x, y좌표를 바탕으로 점을 생성합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 점의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 점의 y좌표 (meters)
   */
  static RelPoint fromRelXY(double rel_x, double rel_y)
  {
    RelPoint point(0, 0);
    Feature::setRelXY(&point, rel_x, rel_y);
    return point;
  }

  /**
   * @returns Collision radius of the ball (meters)
   */
  double getCollisionRadius() const override
  {
    return 0;
  }
};

#endif  // DATA_INTEGRATE_FEATURES_REL_POINT_H
