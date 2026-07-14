import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('scout_navigation')

    nav2_param_path = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

    map_file = os.path.join(
        pkg_share,
        'maps',
        '4floor.yaml'
    )

    # =========================
    # Map Server
    # =========================

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

    # =========================
    # AMCL
    # =========================

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # Planner Server
    # =========================

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # Controller Server
    # =========================

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # BT Navigator
    # =========================

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # Behavior Server
    # =========================

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # Waypoint Follower
    # =========================

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_param_path]
    )

    # =========================
    # Lifecycle Manager
    # =========================

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

    # =========================
    # RViz
    # =========================

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
        rviz_node
    ])
