#ifndef DATA_INTEGRATE_FEATURES_BALL_H
#define DATA_INTEGRATE_FEATURES_BALL_H

#include "./feature.h"

/**
 * 공의 색깔을 나타내는 enumeration입니다.
 */

enum class BallColor
{
  Blue,
  Red,
  Green,
};

/**
 * 공 하나의 위치를 나타냅니다.
 */
class Ball : public Feature
{
public:
  /**
   * 로봇으로부터 공까지의 거리와 각도를 바탕으로 공을 생성합니다.
   *
   * @param color     공의 색깔
   * @param distance  로봇으로부터 공까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 공의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   */
  Ball(BallColor color, double distance, double angle) : color_(color), Feature(distance, angle)
  {
  }

  /**
   * 로봇 좌표계 상의 공의 x, y좌표를 바탕으로 공을 생성합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 공의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 공의 y좌표 (meters)
   */
  static Ball fromRelXY(BallColor color, double rel_x, double rel_y)
  {
    Ball ball(color, 0, 0);
    Feature::setRelXY(&ball, rel_x, rel_y);
    return ball;
  }

  /**
   * 공의 색깔을 구합니다.
   */
  BallColor getColor() const
  {
    return color_;
  }

  /**
   * @returns Collision radius of the ball (meters)
   */
  double getCollisionRadius() const override
  {
    return 0.07;
  }

private:
  BallColor color_;
};

#endif  // DATA_INTEGRATE_FEATURES_BALL_H
