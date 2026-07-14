from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'scout_exploration'
    )

    params_file = os.path.join(
        pkg_share,
        'config',
        'frontier_params.yaml'
    )

    frontier_node = Node(
        package='scout_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        frontier_node
    ])
