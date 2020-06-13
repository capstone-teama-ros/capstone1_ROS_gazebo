#include "data_integrate/tasks/task.h"

TaskResult Task::tick(Blackboard &blackboard)
{
  ROS_DEBUG("%s::tick()", name());
  return doTick(blackboard);
}

void Task::halt(Blackboard &blackboard)
{
  ROS_DEBUG("%s::halt()", name());
  doHalt(blackboard);
}
