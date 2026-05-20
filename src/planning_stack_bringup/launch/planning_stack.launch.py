from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                "launch",
                launch_file_name,
            ])
        )
    )


def generate_launch_description():
    carla_root = LaunchConfiguration("carla_root")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "carla_root",
            description="Path to the CARLA root directory that contains CarlaUE4.sh",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the planning stack",
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("planning_stack_bringup"),
                "rviz",
                "planning_stack.rviz",
            ]),
            description="RViz config file",
        ),

        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([
                    carla_root,
                    "CarlaUE4.sh",
                ]),
                "-RenderOffScreen",
            ],
            cwd=carla_root,
            output="screen",
        ),

        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("carla_ros_bridge"),
                            "carla_ros_bridge_with_example_ego_vehicle.launch.py",
                        ])
                    )
                ),
            ],
        ),

        TimerAction(
            period=20.0,
            actions=[
                include_launch(
                    "hd_map",
                    "hd_map_visualizer.launch.py",
                ),

                include_launch(
                    "mission_planner",
                    "mission_planner.launch.py",
                ),

                include_launch(
                    "behavior_planner",
                    "behavior_planner.launch.py",
                ),

                include_launch(
                    "local_planner",
                    "local_planner.launch.py",
                ),

                include_launch(
                    "velocity_profiler",
                    "velocity_profiler.launch.py",
                ),

                include_launch(
                    "controller",
                    "controller.launch.py",
                ),

                Node(
                    condition=IfCondition(use_rviz),
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                ),
            ],
        ),
    ])