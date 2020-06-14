#ifndef DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_ONCE_H
#define DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_ONCE_H

#include "./task.h"

class PeriodicSearchMoveOnce : public Task
{
public:
  const char *name() const override
  {
    return "PeriodicSearchMoveOnce";
  }

  PeriodicSearchMoveOnce();

private:
  TaskList subtasks_;
  TaskListIter current_subtask_;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_ONCE_H
