#ifndef DATA_INTEGRATE_TASK_EXECUTOR_H
#define DATA_INTEGRATE_TASK_EXECUTOR_H

#include <memory>
#include "./features/visible_feature_manager.h"
#include "./simple_wheel_controller.h"
#include "./tasks/task.h"

/**
 * 작업을 실행하고, 작업 상태의 전환을 관리합니다.
 */
class TaskExecutor
{
public:
  /**
   * @param visible_features 가장 최근에 관측된 지형지물 정보
   */
  TaskExecutor(const VisibleFeatureManager &visible_features) : visible_features_(visible_features)
  {
  }

  /**
   * 현재 실행 중인 작업의 상태를 업데이트하고, 작업의 결정에 따라 바퀴 컨트롤러에 메시지를 전파합니다.
   * 매 루프마다 호출해야 합니다.
   *
   * @param time_passed_after_last  @c updateTask() 를 마지막으로 호출한 이후, 실제로 지난 시간 (seconds)
   * @param time_until_next   @c updateTask() 를 다음 호출할 때까지 예상되는 시간 간격 (seconds)
   * @param wheel_controller  로봇을 운전하기 위해 사용할 바퀴 컨트롤러
   */
  void runTaskInLoop(double time_passed_after_last, double time_until_next, SimpleWheelController &wheel_controller);

  /**
   * 현재 실행 중인 작업을 중지하고 새로운 작업을 강제로 주입합니다.
   * 주의: 테스트용 메소드입니다.
   *
   * @param task  새로 실행할
   */
  void overrideTask(Task::TaskPtr task);

private:
  /// 현재 실행 중인 작업
  std::unique_ptr<Task> task_;
  const VisibleFeatureManager &visible_features_;
};

#endif  // DATA_INTEGRATE_TASK_EXECUTOR_H
