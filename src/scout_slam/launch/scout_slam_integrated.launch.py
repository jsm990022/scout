import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    scout_base_share = get_package_share_directory('scout_base')
    scout_slam_share = get_package_share_directory('scout_slam')

    sensor_integrated_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                scout_base_share,
                'launch',
                'scout_sensor_integrated.launch.py'
            )
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                scout_slam_share,
                'launch',
                'new_scout_slam.launch.py'
            )
        )
    )

    cmd_vel_mux_node = Node(
        package='scout_navigation',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen'
    )

    safety_stop_node = Node(
        package='scout_navigation',
        executable='safety_stop_node',
        name='safety_stop_node',
        output='screen'
    )

    return LaunchDescription([
        sensor_integrated_launch,
        slam_launch,
        cmd_vel_mux_node,
        safety_stop_node
    ])
