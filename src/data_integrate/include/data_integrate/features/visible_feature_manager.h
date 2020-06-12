#ifndef DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H
#define DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H

#include <core_msgs/ball_position.h>
#include <sensor_msgs/LaserScan.h>
#include <unordered_map>
#include <vector>
#include "./ball.h"
#include "./feature_manager.h"
#include "./rel_point.h"

class VisibleFeatureManager : public FeatureManager
{
public:
  /**
   * 카메라의 topic에 subscribe하여 데이터를 받기 위한 callback입니다.
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

  using BallCollection = std::unordered_map<FeatureId, Ball>;
  using LidarPointCollection = std::vector<RelPoint>;

  /**
   * 가장 최근에 카메라로 관측한 공의 목록을 가져옵니다.
   */
  const BallCollection& getBalls() const
  {
    return balls_;
  }

  /**
   * 가장 최근에 LIDAR로 관측한 점의 목록을 가져옵니다.
   */
  const LidarPointCollection& getLidarPoints() const
  {
    return lidar_points_;
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
   * @param ball 기억할 공의 정보
   * @returns 새로운 공에 할당한 ID값
   */
  FeatureId addBall(const Ball& ball);

  /**
   * 기억 중인 모든 feature 정보를 지웁니다.
   */
  void clearAllFeatures();

  /**
   * 입력한 feature ID를 이 feature manager가 이미 사용 중인지 확인합니다.
   */
  bool isFeatureIdInUse(FeatureId id) const override;

private:
  BallCollection balls_;
  LidarPointCollection lidar_points_;
  bool is_blue_ball_captured_ = false;
};

#endif  // DATA_INTEGRATE_FEATURES_VISIBLE_FEATURE_MANAGER_H
