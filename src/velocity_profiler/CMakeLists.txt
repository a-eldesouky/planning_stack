cmake_minimum_required(VERSION 3.10)
project(velocity_profiler)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra)

find_package(ament_cmake            REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(nav_msgs               REQUIRED)
find_package(geometry_msgs          REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(visualization_msgs     REQUIRED)
find_package(planning_msgs          REQUIRED)
find_package(tf2                    REQUIRED)
find_package(tf2_ros                REQUIRED)
find_package(tf2_geometry_msgs      REQUIRED)

add_library(velocity_profiler_lib SHARED
  src/velocity_profiler.cpp
)

target_include_directories(velocity_profiler_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(velocity_profiler_lib PUBLIC cxx_std_17)

add_executable(velocity_profiler_node
  src/velocity_profiler_node.cpp
)

target_link_libraries(velocity_profiler_node
  velocity_profiler_lib
)

ament_target_dependencies(velocity_profiler_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  planning_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_profiler tests/test_velocity_profiler.cpp)
  target_link_libraries(test_velocity_profiler velocity_profiler_lib)
endif()

install(TARGETS
  velocity_profiler_lib
  velocity_profiler_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
