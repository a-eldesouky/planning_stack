cmake_minimum_required(VERSION 3.10)
project(behavior_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(planning_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(derived_object_msgs REQUIRED)


set(SOURCES
  src/behavior_context.cpp
  src/behavior_planner_node.cpp
  src/conditions.cpp
  src/actions.cpp
)

add_executable(behavior_planner_node ${SOURCES})

target_include_directories(behavior_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
  planning_msgs
  behaviortree_cpp
  carla_msgs
  derived_object_msgs
)

install(TARGETS behavior_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()