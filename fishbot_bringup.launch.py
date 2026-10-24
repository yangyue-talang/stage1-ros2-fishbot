#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = DeclareLaunchArgument('map', default_value='/home/y/map.yaml')
    params_file = DeclareLaunchArgument(
        'params_file', default_value='/home/y/maps/nav2_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('fishbot_description'),
                                  'launch', 'gazebo.launch.py'])
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                  'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        map_file,
        params_file,
        gazebo,
        TimerAction(period=25.0, actions=[nav2]),
    ])
