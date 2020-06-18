#include "data_integrate/tasks/master_task.h"

#include <ros/ros.h>

TaskResult MasterTask::doTick(Blackboard &blackboard)
{
  if (is_line_tracer_)
  {
    // Balls are visible
    if (blackboard.visible_features_.getBalls().size() > 0)
    {
      ROS_INFO("%s: LineTracer succeeded!", name());
      line_tracer_task_.halt(blackboard);
      is_line_tracer_ = false;
    }
    else
    {
      auto result = line_tracer_task_.tick(blackboard);
      // 모두 성공할 때까지 순차적으로 실행
      if (result == TaskResult::Success)
      {
        // This never happens, because the LineTracer task never terminates by itself!
        ROS_WARN("%s: This should never be reached", name());
        halt(blackboard);
        return TaskResult::Failure;
      }
      else if (result == TaskResult::Failure)
      {
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
  }

  auto result = ball_harvester_task_.tick(blackboard);
  // 모두 성공할 때까지 순차적으로 실행
  if (result == TaskResult::Success)
  {
    ROS_INFO("%s: BallHarvester succeeded!", name());
    halt(blackboard);
    return TaskResult::Success;
  }
  else if (result == TaskResult::Failure)
  {
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

void MasterTask::doHalt(Blackboard &blackboard)
{
  is_line_tracer_ = true;
  line_tracer_task_.halt(blackboard);
  ball_harvester_task_.halt(blackboard);
}
