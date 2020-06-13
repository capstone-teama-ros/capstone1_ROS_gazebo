#ifndef DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_REPEAT_H
#define DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_REPEAT_H

#include "./task.h"

class PeriodicSearchMoveRepeat : public Task
{
public:
  const char *name() const override
  {
    return "PeriodicSearchMoveRepeat";
  }

  PeriodicSearchMoveRepeat();

private:
  TaskPtr subtask_;
  int subtask_repeat_count_ = 0;
  int subtask_repeat_max_ = 5;

  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;
};

#endif  // DATA_INTEGRATE_TASKS_PERIODIC_SEARCH_MOVE_REPEAT_H
