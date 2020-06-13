#include "data_integrate/tasks/move_distance.h"

#include <algorithm>
#include <cmath>

constexpr double MoveDistance::MAX_SPEED;
constexpr double MoveDistance::THRESHOLD;

void MoveDistance::doHalt(Blackboard &blackboard)
{
  amount_moved_ = 0;
  is_first_tick_ = true;
}

TaskResult MoveDistance::doTick(Blackboard &blackboard)
{
  static_assert(MAX_SPEED >= 0);
  static_assert(THRESHOLD >= 0);

  if (is_first_tick_)
  {
    is_first_tick_ = false;
  }
  else
  {
    // 이동한 거리를 업데이트
    // TODO: 실제 이동 거리를 사용할 수 있으면 좋겠다.
    // TODO: 직진하다가 경로가 틀어질 경우는 어떻게?
    amount_moved_ += blackboard.getTimeSinceLastTick() * blackboard.getRobotLinearSpeed();
  }

  // 원하는 거리까지 이동했으면 성공
  auto difference = std::abs(distance_ - amount_moved_);
  if (difference <= THRESHOLD)
  {
    halt(blackboard);
    return TaskResult::Success;
  }

  // 알맞은 이동 속도를 계산합니다.
  auto speed = std::min(difference / blackboard.getTimeUntilNextTick(), MAX_SPEED);
  if (distance_ < 0)
  {
    speed = -speed;
  }
  blackboard.wheel_controller_.moveLinear(speed);
  return TaskResult::Running;
}
