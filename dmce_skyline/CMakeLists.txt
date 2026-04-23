cmake_minimum_required(VERSION 3.0.2)
project(dmce_skyline)

add_compile_options(-std=c++17)

# 1. Add cv_bridge to catkin and find OpenCV separately
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  grid_map_ros
  dmce_msgs
  dmce_core
  dmce_mcplanner
  cv_bridge            # <--- Ensure this is here
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# 2. Declare the message files
add_message_files(
  FILES
  SkylineCommand.msg
)

# 3. Generate the messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skyline_local_planner
  CATKIN_DEPENDS 
    roscpp 
    dmce_msgs 
    dmce_mcplanner 
    cv_bridge         # <--- Add to catkin depends
    message_runtime
)

# Include OpenCV and Catkin directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # <--- REQUIRED for OpenCV headers
)

# 4. Build the Local Planner Library
add_library(skyline_local_planner
  src/SkylineLocalPlanner.cpp
  src/SkylineMCTree.cpp
)

add_dependencies(skyline_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(skyline_local_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}    # <--- Link OpenCV here if needed
)

# 5. Build the Central Skyline Server Node
add_executable(skyline_server_node 
  src/SkylineServer.cpp
)

add_dependencies(skyline_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(skyline_server_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}    # <--- REQUIRED for imwrite/circle functions
  skyline_local_planner
)