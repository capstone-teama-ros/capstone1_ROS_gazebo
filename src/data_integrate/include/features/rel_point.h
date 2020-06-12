#ifndef DATA_INTEGRATE_FEATURES_REL_POINT_H
#define DATA_INTEGRATE_FEATURES_REL_POINT_H

/**
 * 로봇 좌표계 상의 점 하나의 위치를 나타냅니다.
 */
class RelPoint
{
public:
  /**
   * @param distance 로봇으로부터 점까지의 거리 (meters). 0 이상의 값이어야 합니다.
   * @param angle     로봇이 바라보는 방향(y축)과 점의 위치 벡터가 이루는 각 (radians).
   *                  범위는 -π ~ π 사이의 값으로 정규화(normalize)되니 직접 정규화하지 않아도 됩니다.
   */
  RelPoint(double distance, double angle);

  /**
   * 로봇 좌표계 상의 점의 x, y좌표를 바탕으로 점을 생성합니다.
   *
   * @param rel_x 로봇 좌표계 위에서 점의 x좌표 (meters)
   * @param rel_y 로봇 좌표계 위에서 점의 y좌표 (meters)
   */
  static RelPoint fromRelXY(double rel_x, double rel_y);

  /**
   * @returns 로봇 좌표계 위에서 점의 x좌표 (meters)
   */
  double getRelX() const
  {
    return rel_x_;
  }

  /**
   * @returns 로봇 좌표계 위에서 점의 y좌표 (meters)
   */
  double getRelY() const
  {
    return rel_y_;
  }

  /**
   * @returns 로봇으로부터 점까지의 거리 (meters)
   */
  double getDistance() const
  {
    return distance_;
  }

  /**
   * 로봇이 바라보는 방향(y축)과, 점의 위치 벡터 사이의 각을 구합니다. (radians)
   * 범위는 -π ~ π 입니다.
   * 양수일 경우 반시계 방향, 음수일 경우 시계 방향에 있습니다.
   */
  double getAngle() const
  {
    return angle_;
  }

private:
  /**
   * 다른 생성자에서 호출하기 위한 용도로 만든 생성자입니다.
   * 모든 값을 그대로 저장합니다.
   */
  RelPoint(double distance, double angle, double rel_x, double rel_y)
    : distance_(distance), angle_(angle), rel_x_(rel_x), rel_y_(rel_y){};

  double distance_;
  double angle_;
  double rel_x_;
  double rel_y_;
};

#endif  // DATA_INTEGRATE_FEATURES_REL_POINT_H
