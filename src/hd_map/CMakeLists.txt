cmake_minimum_required(VERSION 3.10)
project(hd_map)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake      REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(nlohmann_json     REQUIRED)

add_library(hd_map_lib SHARED
    src/hd_map.cpp
)

target_include_directories(hd_map_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(hd_map_lib PUBLIC nlohmann_json::nlohmann_json)

add_executable(hd_map_visualizer_node
    src/hd_map_visualizer_node.cpp
)

target_include_directories(hd_map_visualizer_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hd_map_visualizer_node
    hd_map_lib
)

ament_target_dependencies(hd_map_visualizer_node
    rclcpp
    visualization_msgs
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_index_cpp   REQUIRED)

    ament_add_gtest(test_hd_map_node
        tests/test_hd_map_node.cpp
    )

    target_link_libraries(test_hd_map_node
        hd_map_lib
    )

    ament_target_dependencies(test_hd_map_node
        rclcpp
        ament_index_cpp
    )
endif()

install(TARGETS hd_map_lib
    EXPORT  hd_map_libTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS hd_map_visualizer_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY maps/
    DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    OPTIONAL
)

ament_export_targets(hd_map_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(hd_map_lib)
ament_export_dependencies(nlohmann_json)

ament_package()