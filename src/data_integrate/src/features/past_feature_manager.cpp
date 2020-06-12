#include "data_integrate/features/past_feature_manager.h"

#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <random>

PastFeatureManager::PastFeatureManager(double decay_rate) : decay_rate_(decay_rate)
{
  ROS_ASSERT_MSG(0 <= decay_rate && decay_rate <= 1, "Decay rate out of range: %f", decay_rate);
}

const PastFeatureManager::BallCollection& PastFeatureManager::getBalls() const
{
  return balls_;
}

PastFeatureManager::FeatureId PastFeatureManager::addBall(const PastBall& ball)
{
  FeatureId feature_id;
  do
  {
    feature_id = generateId();
  } while (isFeatureIdInUse(feature_id));

  balls_.emplace(feature_id, ball);
}

const PastBall* PastFeatureManager::getBall(FeatureId id) const
{
  auto ball = balls_.find(id);
  if (ball == balls_.end())
  {
    return nullptr;
  }
  return &ball->second;
}

void PastFeatureManager::clearAllFeatures()
{
  balls_.clear();
}

void PastFeatureManager::updateReliability(double time_passed)
{
  ROS_ASSERT(time_passed >= 0);
  auto decay_amount = std::pow(decay_rate_, time_passed);

  for (auto& ball_pair : balls_)
  {
    auto reliability = ball_pair.second.getReliability() * decay_amount;
    ball_pair.second = PastBall(ball_pair.second, reliability);
  }
}

void PastFeatureManager::translateObjectsXY(double x, double y)
{
  for (auto& ball_pair : balls_)
  {
    auto& ball = ball_pair.second;
    ball_pair.second =
        PastBall::fromRelXY(ball.getColor(), ball.getRelX() + x, ball.getRelY() + y, ball.getReliability());
  }
}

void PastFeatureManager::translateSelfXY(double x, double y)
{
  translateObjectsXY(-x, -y);
}

void PastFeatureManager::rotateObjectsZ(double angle)
{
  for (auto& ball_pair : balls_)
  {
    auto& ball = ball_pair.second;
    ball_pair.second = PastBall(ball.getColor(), ball.getDistance(), ball.getAngle() + angle, ball.getReliability());
  }
}

void PastFeatureManager::rotateSelfZ(double angle)
{
  rotateObjectsZ(-angle);
}

PastFeatureManager::FeatureId PastFeatureManager::generateId()
{
  using DistType = std::uniform_int_distribution<FeatureId>;
  using FeatureIdLimits = std::numeric_limits<FeatureId>;

  // 이 static 변수들은 헤더 파일에서 클래스를 선언하는 곳에 넣어도 되지만,
  // 클래스 자체의 정의와는 무관하기 때문에 여기에 넣었습니다.
  static std::default_random_engine rng;
  static DistType feature_id_dist(FeatureIdLimits::min(), FeatureIdLimits::max());
  static bool is_initialized = false;

  // RNG 초기 설정
  if (!is_initialized)
  {
    std::random_device rd;
    rng.seed(rd());
    is_initialized = true;
  }

  return feature_id_dist(rng);
}

bool PastFeatureManager::isFeatureIdInUse(FeatureId id) const
{
  if (balls_.find(id) != balls_.end())
  {
    return true;
  }

  return false;
}
