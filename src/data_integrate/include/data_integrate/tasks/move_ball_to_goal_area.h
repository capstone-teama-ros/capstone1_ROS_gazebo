#ifndef DATA_INTEGRATE_TASKS_MOVE_BALL_TO_GOAL_AREA_H
#define DATA_INTEGRATE_TASKS_MOVE_BALL_TO_GOAL_AREA_H

#include "./task.h"

#include "./avoid_obstacles_to_ball.h"

/**
 * Moves the captured blue ball to the goal area (in front of the goal).
 * Fails if the robot loses the captured ball at any point.
 */
class MoveBallToGoalArea : public Task
{
public:
  const char *name() const override
  {
    return "MoveBallToGoalArea";
  }

  MoveBallToGoalArea() : avoidance_task_(BallColor::Green)
  {
  }

private:
  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;

  AvoidObstaclesToBall avoidance_task_;
};

#endif  // DATA_INTEGRATE_TASKS_MOVE_BALL_TO_GOAL_AREA_H
