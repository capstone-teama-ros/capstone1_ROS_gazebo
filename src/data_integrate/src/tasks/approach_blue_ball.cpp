#include "data_integrate/tasks/approach_blue_ball.h"

#include <angles/angles.h>

TaskResult ApproachBlueBall::doTick(Blackboard& blackboard)
{
  const double MIN_DISTANCE = 0.3;                         // meters
  const double SPEED = 0.3;                                // m/s
  const double ALIGN_THRESHOLD = angles::from_degrees(5);  // radians

  auto& vf = blackboard.visible_features_;

  auto blue_balls = blackboard.visible_features_.getBalls(BallColor::Blue);
  auto nearest_ball = std::min_element(blue_balls.begin(), blue_balls.end(),
                                       [](const Ball& a, const Ball& b) { return a.getDistance() < b.getDistance(); });

  // Abort if there is no blue ball in sight
  if (nearest_ball == blue_balls.end())
  {
    ROS_WARN("%s: No blue ball in sight", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  // Abort if the nearest blue ball is not aligned properly
  if (std::abs(nearest_ball->getAngle()) >= ALIGN_THRESHOLD)
  {
    halt(blackboard);
    return TaskResult::Failure;
  }

  // Finish if the blue ball is close enough
  if (nearest_ball->getDistance() <= MIN_DISTANCE)
  {
    halt(blackboard);
    return TaskResult::Success;
  }

  blackboard.wheel_controller_.moveLinear(SPEED);
  return TaskResult::Running;
}

void ApproachBlueBall::doHalt(Blackboard& blackboard)
{
  blackboard.wheel_controller_.stop();
}
