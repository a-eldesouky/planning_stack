cmake_minimum_required(VERSION 3.10)
project(controller)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(planning_msgs REQUIRED)

add_library(controller_lib
  src/vehicle_model.cpp
  src/pure_pursuit_controller.cpp
)

target_include_directories(controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(controller_lib
  rclcpp
  nav_msgs
  carla_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  planning_msgs
)

add_executable(controller_node
  src/controller_node.cpp
)

target_link_libraries(controller_node
  controller_lib
)

ament_target_dependencies(controller_node
  rclcpp
  nav_msgs
  carla_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  planning_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller tests/test_controller.cpp)
  target_link_libraries(test_controller controller_lib)
endif()

install(TARGETS controller_lib controller_node
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
