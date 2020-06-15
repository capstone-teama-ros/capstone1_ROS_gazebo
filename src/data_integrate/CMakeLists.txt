cmake_minimum_required(VERSION 2.8.3)
project(data_integrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
)

find_package(OpenCV REQUIRED )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  ${PROJECT_NAME}_features_lib
  ${PROJECT_NAME}_tasks_lib
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

#### Build: Libraries ####

add_library(${PROJECT_NAME}_features_lib
  src/features/ball.cpp
  src/features/past_ball.cpp
  src/features/past_feature_manager.cpp
  src/features/rel_point.cpp
  src/features/visible_feature_manager.cpp
)
add_dependencies(${PROJECT_NAME}_features_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME}_tasks_lib
  src/tasks/blue_ball_capture_task.cpp
  src/tasks/blue_ball_return_task.cpp
  src/tasks/blue_ball_search_task.cpp
  src/tasks/goal_post_search_task.cpp
  src/tasks/move_distance.cpp
  src/tasks/periodic_search_move_once.cpp
  src/tasks/periodic_search_move_repeat.cpp
  src/tasks/task.cpp
  src/tasks/turn_angle.cpp
)
add_dependencies(${PROJECT_NAME}_tasks_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#### Build: Executables ####

add_executable(data_integration_node
  src/data_integration.cpp
  src/blackboard.cpp
  src/simple_wheel_controller.cpp
  src/task_executor.cpp
)
add_dependencies(data_integration_node core_msgs_generate_messages_cpp)
target_link_libraries(data_integration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${PROJECT_NAME}_features_lib
  ${PROJECT_NAME}_tasks_lib
)

add_executable(data_show_node
  src/data_show.cpp
  src/robot_drawing.cpp
)
add_dependencies(data_show_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(data_show_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${PROJECT_NAME}_features_lib
)

add_executable(test_simple_wheel_controller_node
  src/test_simple_wheel_controller_node.cpp
  src/simple_wheel_controller.cpp
)
add_dependencies(test_simple_wheel_controller_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_simple_wheel_controller_node
  ${catkin_LIBRARIES}
)

add_executable(test_stationary_align_driver_node
  src/test_stationary_align_driver_node.cpp
  src/stationary_align_driver.cpp
  src/simple_wheel_controller.cpp
)
add_dependencies(test_stationary_align_driver_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_stationary_align_driver_node
  ${catkin_LIBRARIES}
)

add_executable(test_fixed_distance_driver_node
  src/test_fixed_distance_driver_node.cpp
  src/fixed_distance_driver.cpp
  src/simple_wheel_controller.cpp
)
add_dependencies(test_fixed_distance_driver_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_fixed_distance_driver_node
  ${catkin_LIBRARIES}
)

add_executable(test_image_node
  src/test_image_node.cpp
  src/robot_drawing.cpp
)
add_dependencies(test_image_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

add_executable(test_visible_feature_manager_node
  src/test_visible_feature_manager_node.cpp
)
add_dependencies(test_visible_feature_manager_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_visible_feature_manager_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_features_lib
)
