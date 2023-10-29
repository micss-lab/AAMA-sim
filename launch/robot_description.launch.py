import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
import launch
import launch_ros.actions
import xacro

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='aama_robot',
                          choices=['aama_robot'],
                          description='aama_robot Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='aama_robot',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_aama_sim = get_package_share_directory('aama_sim')
    xacro_file = PathJoinSubstitution([pkg_aama_sim,
                                       'urdf',
                                       'common_base.urdf.xacro'])

    namespace = 'aama_robot'

    doc = xacro.process_file(os.path.join(pkg_aama_sim, 'urdf', 'common_base.urdf.xacro'),
                             mappings={'model_name': namespace})

    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  name='robot_state_publisher',
                                  output='both',
                                  parameters=[params])

    # Add nodes to LaunchDescription

    return launch.LaunchDescription([rsp])
