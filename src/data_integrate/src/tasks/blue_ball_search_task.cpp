#include "data_integrate/tasks/blue_ball_search_task.h"

#include <ros/ros.h>
#include <algorithm>
#include "data_integrate/tasks/periodic_search_move_repeat.h"

BlueBallSearchTask::BlueBallSearchTask() : search_task_(new PeriodicSearchMoveRepeat())
{
}

TaskResult BlueBallSearchTask::doTick(Blackboard &blackboard)
{
  // 파란 공을 탐지하면 바로 성공한다
  auto &balls = blackboard.visible_features_.getBalls();
  auto blue_ball =
      std::find_if(balls.begin(), balls.end(), [](const Ball &ball) { return ball.getColor() == BallColor::Blue; });
  if (blue_ball != balls.end())
  {
    halt(blackboard);
    return TaskResult::Success;
  }

  // 파란 공을 찾기 위한 이동을 수행한다.
  auto result = search_task_->tick(blackboard);
  if (result == TaskResult::Success | result == TaskResult::Failure)
  {
    halt(blackboard);
    // 이동을 마쳤다는 것은 파란 공을 탐지하지 못했다는 것이므로 무조건 실패로 처리한다.
    return TaskResult::Failure;
  }
  else if (result == TaskResult::Running)
  {
    return TaskResult::Running;
  }
  else
    ROS_INVALID_TASK_RESULT(result);
}

void BlueBallSearchTask::doHalt(Blackboard &blackboard)
{
  search_task_->halt(blackboard);
}
