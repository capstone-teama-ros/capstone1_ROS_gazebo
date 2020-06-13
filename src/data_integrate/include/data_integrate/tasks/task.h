#ifndef DATA_INTEGRATE_TASKS_TASK_H
#define DATA_INTEGRATE_TASKS_TASK_H

#include <memory>
#include "../features/past_feature_manager.h"
#include "../features/visible_feature_manager.h"
#include "../simple_wheel_controller.h"

/**
 * 모든 작업(Task) 클래스의 기반이 되는 인터페이스 클래스입니다.
 */
class Task
{
public:
  /// @c updateTaskOrMakeNextTask() 가 작업을 리턴할 때 반환하는 포인터의 자료형
  using TaskPtr = std::unique_ptr<Task>;

  /**
   * 이 작업의 현재 상태를 업데이트합니다.
   * 만약 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업을 생성하여 리턴합니다.
   *
   * @param time_passed_after_last  직전의 상태 업데이트를 호출한 이후, 실제로 지난 시간 (seconds)
   * @param visible_features        가장 최근에 관측된 지형지물 정보
   * @param past_features           기억하고 있는 지형지물 정보
   * @returns 현재 작업을 계속 진행해야 할 경우 @c nullptr 를 리턴하면 됩니다.
   * 현재 작업을 중지하고 새로운 작업으로 전환해야 할 경우, 새로운 작업의 클래스를 @c std::make_unique() 로 생성하여
   * 리턴하면 됩니다.
   */
  virtual TaskPtr updateTaskOrMakeNextTask(double time_passed_after_last, const VisibleFeatureManager &visible_features,
                                           const PastFeatureManager &past_features) = 0;

  /**
   * 현재 상태에 알맞게 바퀴 컨트롤러의 상태를 업데이트합니다.
   * 이 함수는 @c updateTaskOrMakeNextTask() 를 호출한 직후에 바로 호출됩니다.
   * 만약 @c updateTaskOrMakeNextTask() 에 의해 작업이 전환되었다면, 새로운 작업이 대신 @c updateWheelController() 를
   * 호출받습니다.
   *
   * @param time_until_next   다음 상태 업데이트까지 예상되는 시간 간격 (seconds)
   * @param wheel_controller  운전 명령을 받을 바퀴 컨트롤러 객체
   */
  virtual void updateWheelController(double time_until_next, SimpleWheelController &wheel_controller) = 0;
};

#endif  // DATA_INTEGRATE_TASKS_TASK_H
