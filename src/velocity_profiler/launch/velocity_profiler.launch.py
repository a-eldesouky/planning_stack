from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    params_file_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("velocity_profiler"),
            "config",
            "velocity_profiler_params.yaml",
        ]),
        description="Full path to velocity_profiler_params.yaml",
    )

    velocity_profiler_node = Node(
        package="velocity_profiler",
        executable="velocity_profiler_node",
        name="velocity_profiler_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        velocity_profiler_node,
    ])
