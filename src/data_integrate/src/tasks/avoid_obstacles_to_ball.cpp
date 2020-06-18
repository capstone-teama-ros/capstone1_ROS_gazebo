#include "data_integrate/tasks/avoid_obstacles_to_ball.h"

#include <ros/ros.h>
#include <limits>
#include "data_integrate/tasks/move_distance.h"
#include "data_integrate/tasks/turn_angle.h"

/**
 * @returns Sum of the collision radius of the @p obstacle and the robot's own radius.
 */
double getFreeRadius(const Feature &obstacle)
{
  // Robot size is approximately 30 cm x 30 cm, so we use a value slightly bigger than sqrt(2) * 0.15 m
  const double ROBOT_FREE_RADIUS = 0.23;  // meters
  return obstacle.getCollisionRadius() + ROBOT_FREE_RADIUS;
}

/**
 * @returns Distance between the collision surface of the @p obstacle and the robot.
 */
double getFreeDistance(const Feature &obstacle)
{
  return obstacle.getDistance() - getFreeRadius(obstacle);
}

/**
 * @returns Free distance between the @p obstacle and the line connecting the robot to the target path.
 * Free distance = (distance between @p obstacle and line) - (obstacle free radius)
 * If the @p obstacle does not block the path at all, returns a very large number.
 */
double computeFreeDistanceToAlignedPath(const Feature &obstacle, const Ball &target)
{
  // Ignore obstacles behind the robot
  if (std::abs(obstacle.getAngle()) >= M_PI / 2)
  {
    return std::numeric_limits<double>::infinity();
  }

  // Ignore obstacles further away than the target
  if (obstacle.getDistance() >= target.getDistance())
  {
    return std::numeric_limits<double>::infinity();
  }

  return std::abs(obstacle.getRelX()) - getFreeRadius(obstacle);
}

/// Checks if the @p obstacle is behind the robot.
bool isBehindRobot(const Feature &obstacle)
{
  return std::abs(obstacle.getAngle()) >= M_PI / 2;
}

TaskResult AvoidObstaclesToBall::doTick(Blackboard &blackboard)
{
  // If there are queued tasks, execute them first
  while (!queued_tasks_.empty())
  {
    auto result = queued_tasks_.front()->tick(blackboard);

    // 모두 성공할 때까지 순차적으로 실행
    if (result == TaskResult::Success)
    {
      queued_tasks_.pop_front();
    }
    else if (result == TaskResult::Failure)
    {
      ROS_WARN("%s: Failure in subtask %s", name(), queued_tasks_.front()->name());
      halt(blackboard);
      return TaskResult::Failure;
    }
    else if (result == TaskResult::Running)
    {
      return TaskResult::Running;
    }
    else
      ROS_INVALID_TASK_RESULT(result);
  }

  // If there are no queued tasks, align with the nearest target-colored ball.
  auto align_result = align_task_.tick(blackboard);
  if (align_result == TaskResult::Success)
  {
    ROS_DEBUG("%s: Align successful", name());
    // Alignment successful, continue to avoidance
  }
  else if (align_result == TaskResult::Failure)
  {
    ROS_WARN("%s: Align failure in %s", name(), align_task_.name());
    halt(blackboard);
    return TaskResult::Failure;
  }
  else if (align_result == TaskResult::Running)
  {
    return TaskResult::Running;
  }
  else
    ROS_INVALID_TASK_RESULT(align_result);

  // Find the nearest target-colored ball
  // Note: If we reached this far, this ball should always exist
  auto target_balls = blackboard.visible_features_.getBalls(ball_color_);
  auto nearest_ball = std::min_element(target_balls.begin(), target_balls.end(),
                                       [](const Ball &a, const Ball &b) { return a.getDistance() < b.getDistance(); });

  // Fail if no target-colored balls are visible (note: this should not happen)
  if (nearest_ball == target_balls.end())
  {
    ROS_ERROR("%s: No target-colored ball in sight", name());
    halt(blackboard);
    return TaskResult::Failure;
  }

  // Build a list of obstacles
  using FeaturePtr = std::unique_ptr<Feature>;
  std::vector<FeaturePtr> obstacles;
  for (auto &red_ball : blackboard.visible_features_.getBalls(BallColor::Red))
  {
    if (computeFreeDistanceToAlignedPath(red_ball, *nearest_ball) < 0)
    {
      obstacles.emplace_back(FeaturePtr(new Ball(red_ball)));
    }
  }
  for (auto &column : blackboard.visible_features_.getColumns())
  {
    if (computeFreeDistanceToAlignedPath(column, *nearest_ball) < 0)
    {
      obstacles.emplace_back(FeaturePtr(new Column(column)));
    }
  }
  for (auto &wall_point : blackboard.visible_features_.getWallPoints())
  {
    if (computeFreeDistanceToAlignedPath(wall_point, *nearest_ball) < 0)
    {
      obstacles.emplace_back(FeaturePtr(new Column(wall_point)));
    }
  }

  auto nearest_obstacle =
      std::min_element(obstacles.begin(), obstacles.end(), [](const FeaturePtr &a, const FeaturePtr &b) {
        return getFreeDistance(*a) < getFreeDistance(*b);
      });

  // Finish if there are no obstacles
  if (nearest_obstacle == obstacles.end())
  {
    ROS_DEBUG("%s: No obstacles!", name());
    halt(blackboard);
    return TaskResult::Success;
  }

  // Avoidance parameters, may need tuning
  double approach_speed = 0.4;                                   // m/s
  double min_safe_distance = getFreeRadius(**nearest_obstacle);  // meters
  double avoid_angle = M_PI / 4;                                 // radians
  double avoid_distance = std::sqrt(2) * min_safe_distance;      // meters
  double recover_angle = -avoid_angle;

  // If the obstacle is too far away, approach it normally
  if (getFreeDistance(**nearest_obstacle) >= min_safe_distance)
  {
    blackboard.wheel_controller_.moveLinear(approach_speed);
    return TaskResult::Running;
  }

  // The obstacle is near. We need to add avoidance tasks to the queue.
  if ((*nearest_obstacle)->getRelX() < 0)
  {
    avoid_angle = -avoid_angle;
    recover_angle = -recover_angle;
  }
  queued_tasks_.push_back(TaskPtr(new TurnAngle(avoid_angle)));
  queued_tasks_.push_back(TaskPtr(new MoveDistance(avoid_distance)));
  queued_tasks_.push_back(TaskPtr(new TurnAngle(recover_angle)));
  return TaskResult::Running;
}

void AvoidObstaclesToBall::doHalt(Blackboard &blackboard)
{
  for (auto &queued_task : queued_tasks_)
  {
    queued_task->halt(blackboard);
  }
  queued_tasks_.clear();

  align_task_.halt(blackboard);
}

const char *AvoidObstaclesToBall::name() const
{
  switch (ball_color_)
  {
    case BallColor::Blue:
      return "AvoidObstaclesToBall[Blue]";
    case BallColor::Red:
      return "AvoidObstaclesToBall[Red]";
    case BallColor::Green:
      return "AvoidObstaclesToBall[Green]";
    default:
      ROS_ASSERT_MSG(0, "Unexpected ball color: %u", static_cast<unsigned int>(ball_color_));
  }
}
