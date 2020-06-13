#ifndef DATA_INTEGRATE_TASKS_TASK_H
#define DATA_INTEGRATE_TASKS_TASK_H

#include <memory>
#include "../blackboard.h"

/**
 * 모든 작업(Task) 클래스의 기반이 되는 인터페이스 클래스입니다.
 */
class Task
{
public:
  /// @c tick() 가 작업을 리턴할 때 반환하는 포인터의 자료형
  using TaskPtr = std::unique_ptr<Task>;

  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 현재 작업을 계속 진행해야 할 경우 @c nullptr 를 리턴하면 됩니다.
   * 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업의 클래스를 @c std::make_unique() 로 생성하여
   * 리턴하면 됩니다.
   */
  virtual TaskPtr tick(Blackboard &blackboard) = 0;
};

#endif  // DATA_INTEGRATE_TASKS_TASK_H
