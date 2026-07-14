import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('scout_navigation')

    nav2_param_path = os.path.join(
        pkg_share,
        'config',
        'nav2_params_safety.yaml'
    )

    map_file = os.path.join(
        pkg_share,
        'maps',
        '4floor.yaml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_param_path,
            {
                'yaml_filename': map_file
            }
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_param_path]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_param_path]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_param_path],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel', '/cmd_vel_nav')
        ]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_param_path]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_param_path]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_param_path]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server',
                    'waypoint_follower'
                ]
            }
        ]
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

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'scout_nav2.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        bt_navigator_node,
        behavior_server_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        cmd_vel_mux_node,
        safety_stop_node,
        rviz_node
    ])
