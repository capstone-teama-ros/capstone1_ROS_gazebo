#ifndef DATA_INTEGRATE_FEATURES_PAST_FEATURE_H
#define DATA_INTEGRATE_FEATURES_PAST_FEATURE_H

/**
 * 과거에 본 사물에 대한 정보를 나타냅니다.
 *
 * 각 사물에 대한 기억의 신뢰성을 reliability라는 값으로 나타냅니다. 최근에 목격한 물체일수록 신뢰도가 높고, 오래 전에
 * 본 물체일수록 신뢰도가 낮습니다.
 * 신뢰도의 범위는 RELIABILITY_MIN <= r <= RELIABILITY_MAX 입니다.
 */
class PastFeature
{
public:
  using Reliability = double;

  static constexpr Reliability RELIABILITY_MAX = 1;
  static constexpr Reliability RELIABILITY_MIN = 0;

  /**
   * @param reliability 이 물체에 대한 기억의 신뢰도
   */
  PastFeature(Reliability reliability = RELIABILITY_MAX) : reliability_(reliability)
  {
  }

  Reliability getReliability() const
  {
    return reliability_;
  }

private:
  Reliability reliability_;
};

#endif  // DATA_INTEGRATE_FEATURES_PAST_FEATURE_H
