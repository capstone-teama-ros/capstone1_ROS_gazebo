#include "data_integrate/tasks/turn_to_nearest_ball.h"

#include <angles/angles.h>
#include <ros/ros.h>
#include <algorithm>
#include "data_integrate/features/ball.h"

TaskResult TurnToNearestBall::doTick(Blackboard& blackboard)
{
  const double MIN_TURN_SPEED = angles::from_degrees(30);  // rad/s; minimum required to overcome friction
  const double MAX_TURN_SPEED = angles::from_degrees(40);  // rad/s
  const double THRESHOLD = angles::from_degrees(5);        // radians; must be large enough to account for decceleration
  const double FINISH_TIME = 1;                            // seconds; time to wait for decceleration

  ROS_ASSERT_MSG(MIN_TURN_SPEED >= 0, "Min turn speed must be 0 or greater (got %f)", MIN_TURN_SPEED);
  ROS_ASSERT_MSG(MAX_TURN_SPEED >= 0, "Max turn speed must be 0 or greater (got %f)", MAX_TURN_SPEED);
  ROS_ASSERT(MIN_TURN_SPEED <= MAX_TURN_SPEED);
  ROS_ASSERT_MSG(THRESHOLD >= 0, "Threshold must be 0 or greater (got %f)", THRESHOLD);
  ROS_ASSERT_MSG(FINISH_TIME >= 0, "Finish time must be 0 or greater (got %f)", FINISH_TIME);

  auto target_balls = blackboard.visible_features_.getBalls(ball_color_);

  auto nearest_ball = std::min_element(target_balls.begin(), target_balls.end(),
                                       [](const Ball& a, const Ball& b) { return a.getDistance() < b.getDistance(); });

  // Fail if no target-colored balls are visible
  if (nearest_ball == target_balls.end())
  {
    ROS_WARN("%s: No target-colored ball in sight", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  auto angle = nearest_ball->getAngle();

  // 원하는 각도까지 회전했으면 성공
  auto angle_difference = std::abs(angle);
  ROS_DEBUG("%s: angle difference: %.3f deg", name(), angles::to_degrees(angle_difference));
  if (angle_difference <= THRESHOLD)
  {
    blackboard.wheel_controller_.stop();

    if (has_been_turning_ && current_finish_timer_ < FINISH_TIME)
    {
      // Wait a bit to ensure proper alignment
      ROS_DEBUG("%s: deccelerating...", name());
      current_finish_timer_ += blackboard.getTimeSinceLastTick();
      return TaskResult::Running;
    }

    halt(blackboard);
    return TaskResult::Success;
  }

  // 알맞은 회전 속도를 계산합니다.
  auto turn_speed = std::max(std::min(angle_difference, MAX_TURN_SPEED), MIN_TURN_SPEED);
  if (angle < 0)
  {
    turn_speed = -turn_speed;
  }
  blackboard.wheel_controller_.turn(turn_speed);
  has_been_turning_ = true;
  return TaskResult::Running;
}

void TurnToNearestBall::doHalt(Blackboard& blackboard)
{
  current_finish_timer_ = 0;
  has_been_turning_ = false;
}

const char* TurnToNearestBall::name() const
{
  switch (ball_color_)
  {
    case BallColor::Blue:
      return "TurnToNearestBall[Blue]";
    case BallColor::Red:
      return "TurnToNearestBall[Red]";
    case BallColor::Green:
      return "TurnToNearestBall[Green]";
    default:
      ROS_ASSERT_MSG(0, "Unexpected ball color: %u", static_cast<unsigned int>(ball_color_));
  }
}
