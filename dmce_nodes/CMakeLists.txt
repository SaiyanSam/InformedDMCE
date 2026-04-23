cmake_minimum_required(VERSION 3.0.2)
project(dmce_nodes)

## Compile as C++17
add_compile_options(-std=c++17)
add_compile_options(-Wno-format-security)

## Find catkin macros and libraries
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  costmap_2d
  smb_ompl_planner

  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization

  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_msgs

  dmce_skyline # <--- Already present, good.
  mh_planner  
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
  roscpp
  std_msgs
  rosgraph_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  message_runtime
  costmap_2d
  smb_ompl_planner

  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization

  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_msgs

  dmce_skyline # <--- Already present.
  mh_planner  
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/GroundTruthMapServer.cpp
  src/SensorEmulatorServer.cpp
  src/NavigationServer.cpp
  src/RobotMapServer.cpp
  src/GlobalMapServer.cpp
  src/PlannerServer.cpp
  src/LoggingServer.cpp
  src/NodeServer.cpp
  src/TransformServer.cpp
  src/RobotMarkerServer.cpp
  src/ConnectivityServer.cpp
  src/FrontierClusterServer.cpp
  src/PathfindingServer.cpp
)

## Add cmake target dependencies of the library
## CRITICAL CHANGE: Added dmce_skyline_generate_messages_cpp
add_dependencies(${PROJECT_NAME}_lib
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 dmce_msgs_generate_messages_cpp
 dmce_skyline_generate_messages_cpp # <--- FIX: Ensures SkylineCommand.h exists
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_nodeMain src/nodeMain.cpp)
add_executable(${PROJECT_NAME}_clock src/clock.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_nodeMain PROPERTIES OUTPUT_NAME nodeMain PREFIX "")
set_target_properties(${PROJECT_NAME}_clock PROPERTIES OUTPUT_NAME clock PREFIX "")

## Add cmake target dependencies of the executable
## CRITICAL CHANGE: Added dmce_skyline_generate_messages_cpp
add_dependencies(${PROJECT_NAME}_nodeMain
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 dmce_msgs_generate_messages_cpp
 dmce_skyline_generate_messages_cpp # <--- FIX
)
add_dependencies(${PROJECT_NAME}_clock
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 dmce_msgs_generate_messages_cpp
 dmce_skyline_generate_messages_cpp # <--- FIX
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_nodeMain
 ${PROJECT_NAME}_lib ${catkin_LIBRARIES}
 ${OMPL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_clock ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_custom_target(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_clock
  ${PROJECT_NAME}_nodeMain
)