#include "data_integrate/features/visible_feature_manager.h"

#include <ros/ros.h>
#include <cmath>

void VisibleFeatureManager::subscribeToLidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // 기존의 점을 모두 지웁니다
  lidar_points_.clear();

  std::vector<int>::size_type i = 0;
  for (auto& range : msg->ranges)
  {
    // 허용 범위 내의 값만 사용합니다.
    // 참고: https://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
    if (msg->range_min <= range && range <= msg->range_max)
    {
      // LIDAR는 x축을 기준으로 반시계 방향으로 각도를 측정합니다.
      // 저희는 y축을 기준으로 측정할 예정이니 90도를 빼야 합니다.
      auto angle = msg->angle_min + msg->angle_increment * i - M_PI / 2;
      lidar_points_.emplace_back(range, angle);
    }

    ++i;
  }
}

void VisibleFeatureManager::subscribeToCamera(const core_msgs::ball_position::ConstPtr& msg)
{
  ROS_ASSERT_MSG(msg->blue_x.size() == msg->blue_y.size(), "Number of X and Y coordinates must be equal (%u != %u)",
                 msg->blue_x.size(), msg->blue_y.size());
  ROS_ASSERT_MSG(msg->red_x.size() == msg->red_y.size(), "Number of X and Y coordinates must be equal (%u != %u)",
                 msg->red_x.size(), msg->red_y.size());
  ROS_ASSERT_MSG(msg->green_x.size() == msg->green_y.size(), "Number of X and Y coordinates must be equal (%u != %u)",
                 msg->green_x.size(), msg->blue_y.size());

  // 기존의 공을 모두 지웁니다
  balls_.clear();

  for (decltype(msg->blue_x)::size_type i = 0; i < msg->blue_x.size(); ++i)
  {
    addBall(Ball::fromRelXY(BallColor::Blue, msg->blue_x[i], msg->blue_y[i]));
  }

  for (decltype(msg->red_x)::size_type i = 0; i < msg->red_x.size(); ++i)
  {
    addBall(Ball::fromRelXY(BallColor::Red, msg->red_x[i], msg->red_y[i]));
  }

  for (decltype(msg->green_x)::size_type i = 0; i < msg->green_x.size(); ++i)
  {
    addBall(Ball::fromRelXY(BallColor::Green, msg->green_x[i], msg->green_y[i]));
  }

  is_blue_ball_captured_ = msg->still_blue;
}

VisibleFeatureManager::FeatureId VisibleFeatureManager::addBall(const Ball& ball)
{
  FeatureId feature_id;
  do
  {
    feature_id = generateId();
  } while (isFeatureIdInUse(feature_id));

  balls_.emplace(feature_id, ball);
}

void VisibleFeatureManager::clearAllFeatures()
{
  balls_.clear();
  lidar_points_.clear();
}

bool VisibleFeatureManager::isFeatureIdInUse(VisibleFeatureManager::FeatureId id) const
{
  if (balls_.find(id) != balls_.end())
  {
    return true;
  }

  return false;
}
