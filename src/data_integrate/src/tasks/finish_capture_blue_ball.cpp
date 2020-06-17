#include "data_integrate/tasks/finish_capture_blue_ball.h"

TaskResult FinishCaptureBlueBall::doTick(Blackboard& blackboard)
{
  const double MOVE_DURATION = 1.5;  // seconds
  const double SPEED = 0.2;          // m/s

  auto& vf = blackboard.visible_features_;

  if (vf.isBlueBallCaptured())
  {
    ROS_INFO("%s: Blue ball captured!", name());
    blackboard.wheel_controller_.stop();
    halt(blackboard);
    return TaskResult::Success;
  }

  if (move_timer_ >= MOVE_DURATION)
  {
    ROS_INFO("%s: Blue ball is not in my grasp", name());
    blackboard.wheel_controller_.stop();
    halt(blackboard);
    return TaskResult::Failure;
  }

  move_timer_ += blackboard.getTimeSinceLastTick();
  blackboard.wheel_controller_.moveLinear(SPEED);
  return TaskResult::Running;
}

void FinishCaptureBlueBall::doHalt(Blackboard& blackboard)
{
  move_timer_ = 0;
}
