from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='aama_robot',
                          description='Ignition model name'),
    DeclareLaunchArgument('robot_count', default_value='4',
                          description='Robot count')
]


def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    robot_name = LaunchConfiguration('robot_name')

    controller_group_actions = GroupAction([
        Node(
            package='aama_sim',
            executable='robot_controller',
            parameters=[
                {
                    'robot_count': robot_count,
                    'robot_name': robot_name
                },
            ],
        ),
        Node(
            package='aama_sim',
            executable='uwb_controller',
            parameters=[
                {
                    'robot_count': robot_count,
                    'robot_name': robot_name
                },
            ],
        ),
        Node(
            package='aama_sim',
            executable='imu_interface',
            parameters=[
                {
                    'robot_count': robot_count,
                    'robot_name': robot_name
                },
            ],
        ),
        Node(
            package='aama_sim',
            executable='sonar_interface',
            parameters=[
                {
                    'robot_count': robot_count,
                    'robot_name': robot_name
                },
            ],
        )
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(controller_group_actions)

    return ld
