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

ROBOT_COUNT = 4
BASE_ROBOT_NAME = 'aama_robot_'
ROBOT_TYPE = 'aama_robot'


def generate_robot_spawn_nodes():
    nodes = []
    distance = 4

    # Define the grid (3x4 to fit up to 12 positions)
    grid = [(x, y) for x in range(0, distance * 3, distance) for y in range(0, distance * 4, distance)]

    # Shuffle the grid to get random positions
    import random
    random.shuffle(grid)

    # Select the required number of coordinates
    coordinates = grid[:ROBOT_COUNT]

    for index, (x, y) in enumerate(coordinates):
        nodes.append(
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=['-name', BASE_ROBOT_NAME + str(index),
                           '-x', str(x),
                           '-y', str(y),
                           '-z', '0.52',
                           '-file', ROBOT_TYPE],
                output='screen'
            )
        )

    return GroupAction(nodes)


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_aama_sim = get_package_share_directory('aama_sim')

    ros_gz_bridge_launch = PathJoinSubstitution([pkg_aama_sim,
                                                 'ros_gz_bridge.launch.py'])

    # Gazebo launch args
    gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=os.path.join(pkg_aama_sim, 'worlds', 'empty.world'),
        description='SDF world file')

    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_gz_bridge_launch]),
        launch_arguments=[
            ('robot_name', ROBOT_TYPE),
            ('namespace', BASE_ROBOT_NAME),
            ('robot_count', str(ROBOT_COUNT))
        ]
    )

    robot_spawn_actions = generate_robot_spawn_nodes()

    multi_robot_group_action = GroupAction([
        # Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
            )
        )
    ])

    ld = LaunchDescription([
        gz_args
    ])

    ld.add_action(multi_robot_group_action)
    ld.add_action(robot_spawn_actions)
    ld.add_action(gz_bridge)
    return ld
