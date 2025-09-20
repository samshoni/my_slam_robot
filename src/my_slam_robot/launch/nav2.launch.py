import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('my_slam_robot')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription([nav2_bringup_launch])

