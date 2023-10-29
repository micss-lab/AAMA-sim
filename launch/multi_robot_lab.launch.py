#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_aama_sim = get_package_share_directory('aama_sim')

    robot_description_launch = PathJoinSubstitution([pkg_aama_sim, 'robot_description.launch.py'])

    # Gazebo launch args
    gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=os.path.join(pkg_aama_sim, 'worlds', 'empty.world'),
        description='SDF world file')

    multi_robot_group_action = GroupAction([
        # Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
            )
        ),

        # Robot 1 description
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([robot_description_launch]),
        #     launch_arguments=[('robot_name', 'aama_robot_1')]
        # ),

        # # Robot 2 description
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([robot_description_launch]),
        #     launch_arguments=[('robot_name', 'aama_robot_2')]
        # ),

        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', 'aama_robot_1',
                       '-x', '0',
                       '-y', '0',
                       '-z', '0.52',
                       '-file', 'aama_robot'],
            output='screen'
        ),
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', 'aama_robot_2',
                       '-x', '0',
                       '-y', '4',
                       '-z', '0.52',
                       '-file', 'aama_robot'],
            output='screen'
        ),
    ])

    ld = LaunchDescription([
        gz_args
    ])
    ld.add_action(multi_robot_group_action)
    return ld
