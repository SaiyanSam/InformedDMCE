cmake_minimum_required(VERSION 3.0.2)
project(mh_planner)

# C++17 is required for std::filesystem
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dmce_core
  dmce_mcplanner
  dmce_msgs
  nav_msgs
  cv_bridge # Added for OpenCV ROS integration
)

# Find OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mh_planner_lib
  CATKIN_DEPENDS roscpp dmce_core dmce_mcplanner dmce_msgs nav_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # Include OpenCV headers
)

add_library(mh_planner_lib
  src/MhDMCTSPlanner.cpp
)

# Link the libraries correctly!
target_link_libraries(mh_planner_lib 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  stdc++fs # Link C++17 filesystem
)

add_dependencies(mh_planner_lib ${catkin_EXPORTED_TARGETS})