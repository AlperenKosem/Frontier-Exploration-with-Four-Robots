cmake_minimum_required(VERSION 3.0.2)
project(explore_map)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  interactive_markers
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  turtlebot3_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(explorer_manager_lib src/exploration_manager.cpp)

add_executable(explorer_manager_node src/exploration_manager_node.cpp)
add_dependencies(explorer_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(explorer_manager_node explorer_manager_lib ${catkin_LIBRARIES})


