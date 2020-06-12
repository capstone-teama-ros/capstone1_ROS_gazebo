#include "../include/features/feature_manager.h"

#include <limits>
#include <random>

FeatureManager::FeatureId FeatureManager::generateId()
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
