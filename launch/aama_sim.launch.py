#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import random
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

ROBOT_COUNT = 2
# BASE_ROBOT_NAME = 'aama_robot_'
# ROBOT_TYPE = 'aama_robot'
# BASE_ROBOT_NAME = '%s_'
ROBOT_TYPE = 'x1'
WORLD_NAME = 'no_roof_small_warehouse.world'
WORLD_NAME_DICT = {
    'warehouse': 'no_roof_small_warehouse.world',
    'house': 'small_house.world',
    'empty': 'empty.world'
}
WORLD_START_COORDS = {
    'house': [(-3.5, -2.3), (-0.4, -2.3), (-0.4, 0), (-3.5, 0.5)],
    'warehouse': [(-2.5, -2.3), (0.4, -2.3), (0.4, 0), (-2.5, 0.5)],
    'empty': [(-4, -4), (0, -4), (0, 0), (-4, 0)]
}


def generate_robot_spawn_nodes(robot_count, world_name, robot_type):
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
    quadrilateral = WORLD_START_COORDS[world_name]

    nodes = []
    distance = 0.3

    # Generate points
    coordinates = generate_points(robot_count, quadrilateral, distance)

    for index, (x, y) in enumerate(coordinates):
        nodes.append(
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=['-name', '%s_' % robot_type + str(index),
                           '-x', str(x),
                           '-y', str(y),
                           '-z', '0.2',
                           '-file', robot_type],
                output='screen'
            )
        )

    return GroupAction(nodes)


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_aama_sim = get_package_share_directory('aama_sim')

    robot_count = LaunchConfiguration('robot_count').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)

    ros_gz_bridge_launch = PathJoinSubstitution([pkg_aama_sim,
                                                 'ros_gz_bridge.launch.py'])

    robot_controller_launch = PathJoinSubstitution([pkg_aama_sim,
                                                    'robot_controller.launch.py'])

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_gz_bridge_launch]),
        launch_arguments=[
            ('robot_name', robot_type),
            ('namespace', '%s_' % robot_type),
            ('robot_count', str(robot_count))
        ]
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_controller_launch]),
        launch_arguments=[
            ('robot_name', robot_type),
            ('robot_count', str(robot_count))
        ]
    )

    robot_spawn_actions = generate_robot_spawn_nodes(int(robot_count), world_name, robot_type)

    gz_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', os.path.join(pkg_aama_sim, 'worlds', WORLD_NAME_DICT[world_name]))
        ]
    )

    return [
        gz_spawn,
        robot_spawn_actions,
        gz_bridge,
        controller
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='warehouse',
            description='AAMA-sim worlds: warehouse, house, empty'),
        DeclareLaunchArgument(
            'robot_count',
            default_value=str(ROBOT_COUNT),
            description='Number of robots to spawn in the simulation'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='x1',
            description='Type of robot to spawn in the simulation'
        ),
        OpaqueFunction(function=launch_setup)
    ])
