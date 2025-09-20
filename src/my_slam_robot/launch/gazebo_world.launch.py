#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_share = FindPackageShare(package='my_slam_robot').find('my_slam_robot')
    
    world_file_name = 'simple_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    urdf_file_name = 'simple_robot.urdf.xacro'
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to \"false\" to run headless.')

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Whether to execute gzclient)')

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient'],
        output='screen')

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_path])}],
        output='screen')

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'simple_robot',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld

