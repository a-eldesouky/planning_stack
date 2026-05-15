from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    params_file_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("behavior_planner"),
            "config",
            "behavior_planner_params.yaml",
        ]),
        description="Full path to behavior_planner_params.yaml",
    )

    behavior_planner_node = Node(
        package="behavior_planner",
        executable="behavior_planner_node",
        name="behavior_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        behavior_planner_node,
    ])
    