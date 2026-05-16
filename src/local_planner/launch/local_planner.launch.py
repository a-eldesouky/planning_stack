from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    params_file_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("local_planner"),
            "config",
            "local_planner_params.yaml",
        ]),
        description="Full path to local_planner_params.yaml",
    )

    local_planner_node = Node(
        package="local_planner",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        local_planner_node,
    ])