cmake_minimum_required(VERSION 2.8.3)
project(lidar_avoid1)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #include
)
add_executable(lidar_avoid_node1 src/lidar_avoid1.cpp)
add_executable(lidar_avoid_node2 src/lidar_avoid2.cpp)


target_link_libraries(lidar_avoid_node1
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)


target_link_libraries(lidar_avoid_node2
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
