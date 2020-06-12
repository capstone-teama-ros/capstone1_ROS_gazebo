#include "data_integrate/features/past_ball.h"

PastBall::PastBall(BallColor color, double distance, double angle, Reliability reliabiility)
  : Ball(color, distance, angle), PastFeature(reliabiility)
{
}

PastBall::PastBall(BallColor color, double distance, double angle) : Ball(color, distance, angle), PastFeature()
{
}

PastBall::PastBall(const Ball &ball, Reliability reliability) : Ball(ball), PastFeature(reliability)
{
}

PastBall::PastBall(const Ball &ball) : Ball(ball), PastFeature()
{
}

PastBall PastBall::fromRelXY(BallColor color, double rel_x, double rel_y, Reliability reliability)
{
  return PastBall(Ball::fromRelXY(color, rel_x, rel_y), reliability);
}

/**
 * 로봇 좌표계 상의 공의 x, y좌표를 바탕으로 공을 생성합니다. 신뢰도는 기본값으로 설정합니다.
 *
 * @param rel_x 로봇 좌표계 위에서 공의 x좌표 (meters)
 * @param rel_y 로봇 좌표계 위에서 공의 y좌표 (meters)
 */
PastBall PastBall::fromRelXY(BallColor color, double rel_x, double rel_y)
{
  return PastBall(Ball::fromRelXY(color, rel_x, rel_y));
}
