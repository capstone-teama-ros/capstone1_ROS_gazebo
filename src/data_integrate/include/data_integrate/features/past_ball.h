#ifndef DATA_INTEGRATE_FEATURES_PAST_BALL_H
#define DATA_INTEGRATE_FEATURES_PAST_BALL_H

#include "./ball.h"
#include "./past_feature.h"

/**
 * 과거에 본 공 하나에 대한 정보를 나타냅니다.
 */
class PastBall : public Ball, public PastFeature
{
public:
  /**
   * 로봇으로부터 공까지의 거리와 각도를 바탕으로 공을 생성합니다.
   *
   * @param color     공의 색깔
   * @param distance  로봇으로부터 공까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 공의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   * @param reliability 공의 위치에 대한 신뢰도.
   */
  PastBall(BallColor color, double distance, double angle, Reliability reliabiility);

  /**
   * 로봇으로부터 공까지의 거리와 각도를 바탕으로 공을 생성합니다. 신뢰도는 기본값으로 설정합니다.
   *
   * @param color     공의 색깔
   * @param distance  로봇으로부터 공까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 공의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   */
  PastBall(BallColor color, double distance, double angle);

  /**
   * 관측한 공의 정보를 바탕으로 공을 생성합니다.
   *
   * @param ball 관측한 공의 정보
   * @param reliability 공의 위치에 대한 신뢰도
   */
  PastBall(const Ball& ball, Reliability reliability);

  /**
   * 관측한 공의 정보를 바탕으로 공을 생성합니다. 신뢰도는 기본값을 사용합니다.
   *
   * @param ball 관측한 공의 정보
   */
  PastBall(const Ball& ball);

  /**
   * 로봇 좌표계 상의 공의 x, y좌표를 바탕으로 공을 생성합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 공의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 공의 y좌표 (meters)
   */
  static PastBall fromRelXY(BallColor color, double rel_x, double rel_y, Reliability reliability);

  /**
   * 로봇 좌표계 상의 공의 x, y좌표를 바탕으로 공을 생성합니다. 신뢰도는 기본값으로 설정합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 공의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 공의 y좌표 (meters)
   */
  static PastBall fromRelXY(BallColor color, double rel_x, double rel_y);
};

#endif  // DATA_INTEGRATE_FEATURES_PAST_BALL_H
