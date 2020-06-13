#include "data_integrate/tasks/task.h"

unsigned int Task::tick_depth_ = 0;
unsigned int Task::halt_depth_ = 0;

const char *toString(TaskResult result)
{
  switch (result)
  {
    case TaskResult::Success:
      return "success";
    case TaskResult::Failure:
      return "failure";
    case TaskResult::Running:
      return "running";
    default:
      ROS_INVALID_TASK_RESULT(result);
  }
}

TaskResult Task::tick(Blackboard &blackboard)
{
  ROS_DEBUG("%*s%s::tick()", tick_depth_ * 2, "", name());

  ++tick_depth_;
  auto result = doTick(blackboard);
  --tick_depth_;

  ROS_DEBUG("%*s\\- %s [%s]", tick_depth_ * 2, "", toString(result), name());
  return result;
}

void Task::halt(Blackboard &blackboard)
{
  ROS_DEBUG("%*s%s::halt()", (tick_depth_ + halt_depth_) * 2, "", name());

  ++halt_depth_;
  doHalt(blackboard);
  --halt_depth_;
}
