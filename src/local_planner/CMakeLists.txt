cmake_minimum_required(VERSION 3.10)
project(local_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)


find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(nav_msgs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(tf2              REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(planning_msgs REQUIRED)

add_library(local_planner_lib STATIC
  src/hybrid_astar.cpp
  src/cubic_spline.cpp
  src/obstacle_processor.cpp
  src/avoidance_latch.cpp
  src/stop_latch.cpp
)

target_include_directories(local_planner_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_planner_lib
  rclcpp
  derived_object_msgs
  planning_msgs
)

add_executable(local_planner_node
  src/local_planner_node.cpp
)

target_include_directories(local_planner_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(local_planner_node
  local_planner_lib
)

ament_target_dependencies(local_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  derived_object_msgs
  tf2
  tf2_geometry_msgs
  planning_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_local_planner tests/test_local_planner.cpp)
  target_link_libraries(test_local_planner local_planner_lib)
endif()

install(TARGETS
  local_planner_lib
  local_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()