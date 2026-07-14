from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='velodyne_to_laserscan',

        remappings=[
            ('cloud_in', '/velodyne_points'),
            ('scan', '/scan')
        ],

        parameters=[{

            # TF
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,

            # 높이 필터
            'min_height': 0.01,
            'max_height': 1.5,

            # 각도
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174,

            # 성능 핵심
            'scan_time': 0.1,

            # 거리
            'range_min': 0.05,
            'range_max': 20.0,

            'use_inf': True,

            # 매우 중요
            'queue_size': 5,

            # 시뮬레이션 사용 안함
            'use_sim_time': False

        }],

        output='screen'
    )

    return LaunchDescription([
        pointcloud_to_laserscan_node
    ])
