#ifndef DATA_INTEGRATE_TASKS_SEQUENCE_H
#define DATA_INTEGRATE_TASKS_SEQUENCE_H

#include "./task.h"

/**
 * Combined task that repeatedly runs a sequence of subtasks.
 * Subtasks that succeed are not tried again.
 * If any of the subtasks fail, this task fails, too.
 */
class SequenceTask : public Task
{
public:
  SequenceTask(int count, ...);

  const char *name() const override
  {
    return "Sequence";
  }

private:
  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 작업을 실행한 결과
   */
  TaskResult doTick(Blackboard &blackboard) override;
  void doHalt(Blackboard &blackboard) override;

  TaskList subtasks_;
  TaskListIter current_subtask_;
};

#endif  // DATA_INTEGRATE_TASKS_SEQUENCE_H
