#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import random
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

ROBOT_COUNT = 3
BASE_ROBOT_NAME = 'aama_robot_'
ROBOT_TYPE = 'aama_robot'
WORLD_NAME = 'small_house.world'


def generate_robot_spawn_nodes():
    def is_inside_polygon(point, polygon):
        """Check if a point is inside a polygon using the ray-casting algorithm."""
        x, y = point
        inside = False
        for i in range(len(polygon)):
            j = (i + 1) % len(polygon)
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
        return inside

    def generate_points(n, polygon, min_distance):
        """Generate n points within a polygon ensuring each is min_distance apart."""
        points = []
        # Calculate bounding box
        xs, ys = zip(*polygon)
        min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)

        while len(points) < n:
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            point = (x, y)
            if is_inside_polygon(point, polygon) and all(
                    np.linalg.norm(np.array(point) - np.array(p)) >= min_distance for p in points):
                points.append(point)
        return points

    # Define the quadrilateral vertices
    quadrilateral = [(-3.5, -2.3), (-0.4, -2.3), (-0.4, 0), (-3.5, 0.5)]  # Example: A square

    nodes = []
    distance = 0.3

    # Generate points
    coordinates = generate_points(ROBOT_COUNT, quadrilateral, distance)

    for index, (x, y) in enumerate(coordinates):
        nodes.append(
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=['-name', BASE_ROBOT_NAME + str(index),
                           '-x', str(x),
                           '-y', str(y),
                           '-z', '0.2',
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

    robot_controller_launch = PathJoinSubstitution([pkg_aama_sim,
                                                    'robot_controller.launch.py'])

    # Gazebo launch args
    gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=os.path.join(pkg_aama_sim, 'worlds', WORLD_NAME),
        description='SDF world file')

    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_gz_bridge_launch]),
        launch_arguments=[
            ('robot_name', ROBOT_TYPE),
            ('namespace', BASE_ROBOT_NAME),
            ('robot_count', str(ROBOT_COUNT))
        ]
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_controller_launch]),
        launch_arguments=[
            ('robot_name', ROBOT_TYPE),
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
    ld.add_action(controller)
    return ld
