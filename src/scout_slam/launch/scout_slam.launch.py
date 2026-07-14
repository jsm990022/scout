import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('scout_slam')
    
    # 1. 내 패키지 안의 기본 yaml 파일 경로
    slam_config_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'scout_slam.rviz')

    # 🌟 [신규] 터미널에서 입력받을 인자(Argument) 선언
    # 뒤에 load_map:=파일명 을 안 쓰면 기본값으로 빈 문자열('')이 들어갑니다.
    load_map_arg = DeclareLaunchArgument(
        'load_map',
        default_value='',
        description='기존 이어서 그릴 맵 파일 이름 (예: bufs)'
    )

    # 터미널 입력값을 변수처럼 바인딩
    load_map_name = LaunchConfiguration('load_map')

    return LaunchDescription([
        # 인자 등록
        load_map_arg,

        # 📌 3D 벨로다인 ➡ 2D 레이저 스캔 변환 노드
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='velodyne_to_laserscan',
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': -0.45,
                'max_height': 0.1,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 30.0,
                'use_inf': True,
                'reliability': 'BEST_EFFORT'
            }]
        ),

        # 📌 slam_toolbox 알고리즘 본체 노드 구동
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_path,          # 기본 yaml 설정 주입
                {
                    'use_sim_time': False,
                    # 🌟 [핵심 치트키] 터미널에 load_map:=bufs라고 치면 
                    # 자동으로 그 경로의 posegraph 파일을 읽어오고, 안 치면 새로 맵을 그립니다!
                    'map_file_name': [os.path.expanduser('~'), '/scout_ws/', load_map_name],
                    'map_start_at_dock': True
                }
            ]
        ),

        # 📌 RViz2 자동 소환 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
