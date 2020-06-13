#ifndef DATA_INTEGRATE_TASKS_TASK_H
#define DATA_INTEGRATE_TASKS_TASK_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include "../blackboard.h"

/**
 * 한 작업(Task)을 한 tick만큼 실행한 뒤의 결과를 나타냅니다.
 */
enum class TaskResult
{
  Failure,  ///< 실패
  Success,  ///< 성공
  Running,  ///< 진행 중
};

// Task의 실행 결과가 잘못되었을 때 디버깅을 위해 쓰는 매크로
#define ROS_INVALID_TASK_RESULT(result) ROS_ASSERT_MSG(0, "Invalid task result: %u", static_cast<unsigned int>(result))

/**
 * 모든 작업(Task) 클래스의 기반이 되는 인터페이스 클래스입니다.
 * Behavior tree의 node를 나타냅니다.
 */
class Task
{
public:
  /// 하위 작업을 관리하기 위한 스마트 포인터의 자료형
  using TaskPtr = std::unique_ptr<Task>;
  using TaskList = std::vector<TaskPtr>;
  using TaskListIter = TaskList::iterator;

  /**
   * 이 작업을 한 tick만큼 실행하여 현재 상태를 업데이트하고 결과를 돌려줍니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   * @returns 작업을 실행한 결과
   */
  virtual TaskResult tick(Blackboard &blackboard) = 0;

  /**
   * 이 작업과 모든 하위 작업을 중지합니다.
   *
   * @param blackboard 작업 실행에 사용할 Blackboard
   */
  virtual void halt(Blackboard &blackboard) = 0;

private:
};

#endif  // DATA_INTEGRATE_TASKS_TASK_H
