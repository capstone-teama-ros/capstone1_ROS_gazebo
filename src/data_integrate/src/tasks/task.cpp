#include "data_integrate/tasks/task.h"

unsigned int Task::tick_depth_ = 0;
unsigned int Task::halt_depth_ = 0;

TaskResult Task::tick(Blackboard &blackboard)
{
  ROS_DEBUG("%*s%s::tick()", tick_depth_ * 2, "", name());

  ++tick_depth_;
  auto result = doTick(blackboard);
  --tick_depth_;

  return result;
}

void Task::halt(Blackboard &blackboard)
{
  ROS_DEBUG("%*s%s::halt()", (tick_depth_ + halt_depth_) * 2, "", name());

  ++halt_depth_;
  doHalt(blackboard);
  --halt_depth_;
}
