cmake_minimum_required(VERSION 3.8)
project(planning_msgs)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RouteWaypoint.msg"
  "msg/Route.msg"
  "msg/BehaviorCommand.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
