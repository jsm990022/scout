import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scout_navigation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),

        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
        ),

        (
            os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))
        ),

        (
            os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))
        ),

    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='i421',
    maintainer_email='i421@todo.todo',

    description='Scout Navigation Package',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'safety_stop_node = scout_navigation.safety_stop_node:main',
            'recovery_behavior_node = scout_navigation.recovery_behavior_node:main',
            'multi_goal_mission_node = scout_navigation.multi_goal_mission_node:main',
            'auto_patrol_node = scout_navigation.auto_patrol_node:main',
            'battery_return_node = scout_navigation.battery_return_node:main',
            'web_goal_manager = scout_navigation.web_goal_manager:main',
            'cmd_vel_mux_node = scout_navigation.cmd_vel_mux_node:main',
        ],
    },
)
