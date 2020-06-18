#include "data_integrate/tasks/kick_ball_into_goal.h"

#include <angles/angles.h>

TaskResult KickBallIntoGoal::doTick(Blackboard& blackboard)
{
  const double KICK_DISTANCE = 0.1;                        // meters
  const double SPEED = 0.4;                                // m/s
  const double ALIGN_THRESHOLD = angles::from_degrees(5);  // radians
  const double KICK_CONFIRM_TIME = 1.5;                    // seconds;

  // Find the nearest green ball
  // Note: If we reached this far, this ball should always exist
  auto goal_balls = blackboard.visible_features_.getBalls(BallColor::Green);
  auto nearest_ball = std::min_element(goal_balls.begin(), goal_balls.end(),
                                       [](const Ball& a, const Ball& b) { return a.getDistance() < b.getDistance(); });

  // Fail if no target-colored balls are visible (note: this should not happen)
  if (nearest_ball == goal_balls.end())
  {
    ROS_ERROR("%s: No green ball in sight", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  if (nearest_ball->getAngle() >= ALIGN_THRESHOLD)
  {
    ROS_ERROR("%s: Not aligned with the green ball", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  if (nearest_ball->getDistance() >= KICK_DISTANCE)
  {
    blackboard.wheel_controller_.moveLinear(SPEED);
    return TaskResult::Running;
  }

  // Finish kick
  blackboard.wheel_controller_.stop();
  if (kick_confirm_timer_ < KICK_CONFIRM_TIME)
  {
    // Wait for the ball to be ejected
    ROS_INFO_COND(kick_confirm_timer_ == 0, "%s: Kicked the ball into the goal", name());
    kick_confirm_timer_ += blackboard.getTimeSinceLastTick();
    return TaskResult::Running;
  }

  if (!blackboard.visible_features_.isBlueBallCaptured())
  {
    ROS_INFO("%s: Goal success!", name());
    halt(blackboard);
    return TaskResult::Success;
  }

  // TODO: Not sure what to do here....
  ROS_ERROR("%s: Ball is stuck in robot", name());
  halt(blackboard);
  return TaskResult::Failure;
}

void KickBallIntoGoal::doHalt(Blackboard& blackboard)
{
  kick_confirm_timer_ = 0;
}
