import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    pkg_share = get_package_share_directory('scout_navigation')



    nav2_param_path = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )



    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',

        parameters=[
            nav2_param_path
        ]
    )



    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',

        parameters=[
            nav2_param_path
        ]
    )



    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',

        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl'
            ]
        }]
    )



    return LaunchDescription([

        map_server_node,

        amcl_node,

        lifecycle_manager_node,

    ])
