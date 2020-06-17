#ifndef DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H
#define DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H

#include <core_msgs/ball_position.h>
#include <core_msgs/line_info.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "./ball.h"
#include "./rel_point.h"

/**
 * 가장 최근에 관측한 정보를 담는 클래스입니다.
 */
class VisibleFeatureManager
{
public:
  /**
   * 카메라의 ball detection topic에 subscribe하여 데이터를 받기 위한 callback입니다.
   *
   * @param msg topic으로부터 받는 데이터 메시지
   */
  void subscribeToCamera(const core_msgs::ball_position::ConstPtr& msg);

  /**
   * LIDAR topic에 subscribe하여 데이터를 받기 위한 callback입니다.
   *
   * @param msg topic으로부터 받는 데이터 메시지
   */
  void subscribeToLidar(const sensor_msgs::LaserScan::ConstPtr& msg);

  using BallCollection = std::vector<Ball>;
  using LidarPointCollection = std::vector<RelPoint>;

  /**
   * IMU topic에 subscribe하여 데이터를 받기 위한 callback입니다.
   *
   * @param msg topic으로부터 받는 데이터 메시지
   */
  void subscribeToImu(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * 카메라의 line tracer topic에 subscribe하여 데이터를 받기 위한 callback입니다.
   *
   * @param msg topic으로부터 받는 데이터 메시지
   */
  void subscribeToLineInfo(const core_msgs::line_info::ConstPtr& msg);

  /**
   * 가장 최근에 카메라로 관측한 공의 목록을 가져옵니다.
   */
  const BallCollection& getBalls() const
  {
    return balls_;
  }

  /**
   * 가장 최근에 카메라로 관측한 공 중에서 특정한 색깔의 공들만 가져옵니다.
   */
  BallCollection getBalls(BallColor color) const;

  /**
   * 가장 최근에 LIDAR로 관측한 점의 목록을 가져옵니다.
   */
  const LidarPointCollection& getLidarPoints() const
  {
    return lidar_points_;
  }

  /**
   * 가장 최근에 LIDAR로 관측한 기둥의 목록을 가져옵니다.
   */
  const LidarPointCollection& getColumns() const
  {
    return columns_;
  }

  /**
   * 현재 파란 공이 포획되어 있는지 확인합니다.
   */
  bool isBlueBallCaptured() const
  {
    return is_blue_ball_captured_;
  }

  /**
   * 새로운 공을 추가합니다.
   *
   * @param ball 관측한 공의 정보
   */
  void addBall(const Ball& ball);

  using ImuVectorT = geometry_msgs::Vector3;

  /**
   * @returns IMU로 가장 최근에 측정한 각속도 벡터 (rad/s)
   */
  const ImuVectorT& getImuAngularVelocity() const
  {
    return imu_angular_velocity_;
  }

  /**
   * @returns IMU로 가장 최근에 측정한 선가속도 벡터 (m/s^2)
   */
  const ImuVectorT& getImuLinearAcceleration() const
  {
    return imu_linear_acceleration_;
  }

  /**
   * @returns 라인트레이서: 검은 줄이 속한 구간 번호 (line_detect.cpp 참고)
   */
  int getTracerSection() const
  {
    return line_tracer_section_;
  }

  /**
   * @returns 라인트레이서: 검은 줄의 bounding box의 x좌표 (픽셀)
   */
  int getTracerBoxX() const
  {
    return line_tracer_box_x_;
  }

  /**
   * @returns 라인트레이서: 검은 줄의 bounding box의 y좌표 (픽셀)
   */
  int getTracerBoxY() const
  {
    return line_tracer_box_y_;
  }

  /**
   * @returns 라인트레이서: 검은 줄의 bounding box의 너비 (픽셀)
   */
  int getTracerBoxWidth() const
  {
    return line_tracer_box_width_;
  }

  /**
   * @returns 라인트레이서: 검은 줄의 bounding box의 높이 (픽셀)
   */
  int getTracerBoxHeight() const
  {
    return line_tracer_box_height_;
  }

  /**
   * @returns 라인트레이서: 이미지의 전체 너비 (픽셀)
   */
  int getTracerImageWidth() const
  {
    return line_tracer_box_width_;
  }

  /**
   * @returns 라인트레이서: 이미지의 전체 높이 (픽셀)
   */
  int getTracerImageHeight() const
  {
    return line_tracer_box_height_;
  }

  /**
   * 기억 중인 모든 feature 정보를 지웁니다.
   */
  void clearAllFeatures();

private:
  BallCollection balls_;
  LidarPointCollection lidar_points_;
  LidarPointCollection columns_;
  bool is_blue_ball_captured_ = false;
  ImuVectorT imu_angular_velocity_;     ///< (x축, y축, z축) (rad/s)
  ImuVectorT imu_linear_acceleration_;  ///< (x, y, z) (m/s^2)
  int line_tracer_section_;
  int line_tracer_box_x_;
  int line_tracer_box_y_;
  int line_tracer_box_width_;
  int line_tracer_box_height_;
  int line_tracer_image_width_;
  int line_tracer_image_height_;
};

#endif  // DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H
