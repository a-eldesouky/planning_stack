cmake_minimum_required(VERSION 3.10)
project(mission_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake        REQUIRED)
find_package(hd_map             REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(planning_msgs      REQUIRED)

add_library(mission_planner_lib SHARED
    src/mission_planner.cpp
)

target_include_directories(mission_planner_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_planner_lib
    hd_map
)

add_executable(mission_planner_node
    src/mission_planner_node.cpp
)

target_include_directories(mission_planner_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(mission_planner_node
    mission_planner_lib
)

ament_target_dependencies(mission_planner_node
    hd_map
    rclcpp
    nav_msgs
    geometry_msgs
    planning_msgs
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_mission_planner
        tests/test_mission_planner.cpp
    )

    target_include_directories(test_mission_planner PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    target_link_libraries(test_mission_planner
        mission_planner_lib
    )

    ament_target_dependencies(test_mission_planner
        hd_map
    )

    # Use the Town01 map installed by the hd_map package (share/hd_map/maps).
    get_filename_component(HD_MAP_SHARE_DIR "${hd_map_DIR}/.." ABSOLUTE)
    target_compile_definitions(test_mission_planner PRIVATE
        TOWN01_MAP_PATH="${HD_MAP_SHARE_DIR}/maps/Town01_hd_map.json"
    )
endif()

install(TARGETS mission_planner_lib
    EXPORT  mission_planner_libTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS mission_planner_node
    DESTINATION lib/${PROJECT_NAME}
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

ament_export_targets(mission_planner_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mission_planner_lib)

ament_package()