from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('hd_map'), 'maps', 'Town01_hd_map.json'
        ]),
        description='Path to the HD map JSON file'
    )

    visualizer_node = Node(
        package='hd_map',
        executable='hd_map_visualizer_node',
        name='hd_map_visualizer_node',
        output='screen',
        parameters=[{
            'map_path':            LaunchConfiguration('map_path'),
            'frame_id':            'map',
            'show_boundaries':     True,
            'show_lane_ids':       True,
            'show_directions':     True,
            'centerline_z_offset': 0.15,
            'republish_period_s':  10.0,
        }]
    )

    return LaunchDescription([
        map_path_arg,
        visualizer_node,
    ])
