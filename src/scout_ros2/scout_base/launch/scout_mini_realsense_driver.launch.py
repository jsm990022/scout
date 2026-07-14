import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                          description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_footprint',
                                                description='Base footprint frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')

    is_scout_mini_arg = DeclareLaunchArgument('is_scout_mini', default_value='true',
                                          description='Scout mini model')
    is_omni_wheel_arg = DeclareLaunchArgument('is_omni_wheel', default_value='false',
                                          description='Scout mini omni-wheel model')

    simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
                                                   description='Whether running with simulator')
    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')
    
    scout_base_node = launch_ros.actions.Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'is_scout_mini': launch.substitutions.LaunchConfiguration('is_scout_mini'),
                'is_omni_wheel': launch.substitutions.LaunchConfiguration('is_omni_wheel'),
                'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
                'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
        }])

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scout_to_camera_tf',
        arguments=['0.15', '0.0', '0.25', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    realsense_share_dir = get_package_share_directory('realsense2_camera')
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_share_dir, 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'pointcloud.enable': 'true',
            'initial_reset': 'true'
        }.items()
    )

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
        scout_base_node,
        camera_tf_node,
        realsense_driver
    ])
