#ifndef DATA_INTEGRATE_TASKS_LINE_TRACER_H
#define DATA_INTEGRATE_TASKS_LINE_TRACER_H

#include "./task.h"

/**
 * 파란 공을 포획한 이후, 공을 넣을 골대의 위치를 탐색하는 작업입니다.
 */
class LineTracer : public Task
{
public:
  const char *name() const override
  {
    return "LineTracer";
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

  enum class LinePosition : int32_t
  {
    None = 3,
    Left = 0,
    Center = 1,
    Right = 2,
  };

  LinePosition line_position_ = LinePosition::None;
};

#endif  // DATA_INTEGRATE_TASKS_LINE_TRACER_H
