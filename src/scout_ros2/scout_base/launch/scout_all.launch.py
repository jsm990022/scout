import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # =========================
    # Launch Arguments
    # =========================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='can0',
        description='CAN bus name'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id'
    )

    base_link_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame id'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic_name',
        default_value='odom',
        description='Odometry topic name'
    )

    is_scout_mini_arg = DeclareLaunchArgument(
        'is_scout_mini',
        default_value='true',
        description='Scout mini model'
    )

    is_omni_wheel_arg = DeclareLaunchArgument(
        'is_omni_wheel',
        default_value='false',
        description='Scout omni wheel model'
    )

    simulated_robot_arg = DeclareLaunchArgument(
        'simulated_robot',
        default_value='false',
        description='Whether using simulator'
    )

    sim_control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50',
        description='Control loop rate'
    )

    # =========================
    # Robot Description
    # =========================

    scout_description_dir = get_package_share_directory('scout_description')

    xacro_file = os.path.join(
        scout_description_dir,
        'urdf',
        'scout_mini',
        'scout_mini.xacro'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    # =========================
    # Robot State Publisher
    # =========================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # =========================
    # Joint State Publisher
    # =========================

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # =========================
    # Velodyne TF
    # =========================

    velodyne_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_static_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'velodyne_link',
            'velodyne'
        ]
    )

    # =========================
    # Scout Base Node
    # =========================

    scout_base_node = launch_ros.actions.Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port_name': LaunchConfiguration('port_name'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name'),
            'is_scout_mini': LaunchConfiguration('is_scout_mini'),
            'is_omni_wheel': LaunchConfiguration('is_omni_wheel'),
            'simulated_robot': LaunchConfiguration('simulated_robot'),
            'control_rate': LaunchConfiguration('control_rate'),
        }]
    )

    # =========================
    # Velodyne Driver
    # =========================

    velodyne_driver_dir = get_package_share_directory('velodyne_driver')

    velodyne_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                velodyne_driver_dir,
                'launch',
                'velodyne_driver_node-VLP16-launch.py'
            )
        ])
    )

    # =========================
    # Velodyne PointCloud
    # =========================

    velodyne_pointcloud_dir = get_package_share_directory('velodyne_pointcloud')

    velodyne_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                velodyne_pointcloud_dir,
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            )
        ])
    )

    # =========================
    # RealSense Driver
    # =========================

    realsense_share_dir = get_package_share_directory('realsense2_camera')

    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                realsense_share_dir,
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'pointcloud.enable': 'true',
            'initial_reset': 'true'
        }.items()
    )

    # =========================
    # Launch Description
    # =========================

    return LaunchDescription([

        use_sim_time_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        is_scout_mini_arg,
        is_omni_wheel_arg,
        simulated_robot_arg,
        sim_control_rate_arg,

        robot_state_publisher_node,
        joint_state_publisher_node,

        velodyne_static_tf,

        scout_base_node,

        velodyne_driver,
        velodyne_transform,

        realsense_driver
    ])
