cmake_minimum_required(VERSION 3.0.2)
project(swarm_metrics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  dmce_msgs
  roslib
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(overlap_logger src/overlap_logger.cpp)

target_link_libraries(overlap_logger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Tell CMake to wait for message headers to be generated first!
add_dependencies(overlap_logger ${catkin_EXPORTED_TARGETS})