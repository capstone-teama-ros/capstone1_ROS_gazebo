#ifndef DATA_INTEGRATE_FEATURES_FEATURE_MANAGER_H
#define DATA_INTEGRATE_FEATURES_FEATURE_MANAGER_H

/**
 * 지형지물(feature)을 관리하는 클래스의 공통 부모 클래스입니다.
 */
class FeatureManager
{
public:
  // 공과 같은 중요한 지형지물을 식별하기 위해 부여하는 고유한 값
  using FeatureId = unsigned int;

  /**
   * 새로운 지형지물을 만들기 위한 ID 값을 생성합니다.
   * 기존의 지형지물이 사용하는 ID값과 겹칠 수 있으니 @c isFeatureIdInUse() 로 검사해야 합니다.
   */
  static FeatureId generateId();

  /**
   * 입력한 feature ID를 이 feature manager가 이미 사용 중인지 확인합니다.
   */
  virtual bool isFeatureIdInUse(FeatureId id) const = 0;
};

#endif  // DATA_INTEGRATE_FEATURES_FEATURE_MANAGER_H
