from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='aama_robot',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='aama_robot',
                          description='Robot namespace'),
    DeclareLaunchArgument('robot_count', default_value='4',
                          description='Robot count')
]


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_count = LaunchConfiguration('robot_count').perform(context)

    cmd_vel_bridge_nodes = []
    pose_bridge_nodes = []

    for i in range(int(robot_count)):
        cmd_vel_bridge_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='cmd_vel_bridge_' + str(i),
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    ['model/',
                     '%s%s' % (namespace, i),
                     '/cmd_vel' +
                     '@geometry_msgs/msg/Twist' +
                     ']ignition.msgs.Twist'],
                ]
            )
        )

    for i in range(int(robot_count)):
        pose_bridge_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='pose_bridge_' + str(i),
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    ['model/',
                     '%s_%s' % (robot_name, i),
                     '/pose_static' +
                     '@tf2_msgs/msg/TFMessage' +
                     '[ignition.msgs.Pose_V'],
                ]
            )
        )

    imu_sonar_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                ['imu' +
                 '@sensor_msgs/msg/Imu' +
                 '[ignition.msgs.IMU'],
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sonar_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                ['sonar/points' +
                 '@sensor_msgs/msg/LaserScan' +
                 '[ignition.msgs.LaserScan'],
            ]
        )]

    return cmd_vel_bridge_nodes + pose_bridge_nodes + imu_sonar_nodes


def generate_launch_description():
    # # lidar bridge
    # lidar_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='lidar_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/link/rplidar_link/sensor/rplidar/scan' +
    #          '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
    #     ],
    #     remappings=[
    #         (['/world/', world,
    #           '/model/', robot_name,
    #           '/link/rplidar_link/sensor/rplidar/scan'],
    #          'scan')
    #     ])
    #
    # # Display message bridge
    # hmi_display_msg_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='hmi_display_msg_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         [namespace, '/hmi/display/raw' +
    #          '@std_msgs/msg/String' +
    #          ']ignition.msgs.StringMsg'],
    #         [namespace, '/hmi/display/selected' +
    #          '@std_msgs/msg/Int32' +
    #          ']ignition.msgs.Int32']
    #     ],
    #     remappings=[
    #         ([namespace, '/hmi/display/raw'],
    #          'hmi/display/_raw'),
    #         ([namespace, '/hmi/display/selected'],
    #          'hmi/display/_selected')
    #     ],
    #     condition=LaunchConfigurationEquals('model', 'standard'))
    #
    # # Buttons message bridge
    # hmi_buttons_msg_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='hmi_buttons_msg_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         [namespace, '/hmi/buttons' +
    #          '@std_msgs/msg/Int32' +
    #          '[ignition.msgs.Int32']
    #     ],
    #     remappings=[
    #         ([namespace, '/hmi/buttons'],
    #          'hmi/buttons/_set')
    #     ],
    #     condition=LaunchConfigurationEquals('model', 'standard'))
    #
    # # Buttons message bridge
    # hmi_led_msg_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='hmi_led_msg_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         [namespace, '/hmi/led/' + led +
    #          '@std_msgs/msg/Int32' +
    #          ']ignition.msgs.Int32'] for led in leds
    #     ],
    #     remappings=[
    #         ([namespace, '/hmi/led/' + led],
    #          'hmi/led/_' + led) for led in leds
    #     ],
    #     condition=LaunchConfigurationEquals('model', 'standard'))
    #
    # # Camera sensor bridge
    # oakd_camera_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='camera_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image' +
    #          '@sensor_msgs/msg/Image' +
    #          '[ignition.msgs.Image'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
    #          '@sensor_msgs/msg/Image' +
    #          '[ignition.msgs.Image'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points' +
    #          '@sensor_msgs/msg/PointCloud2' +
    #          '[ignition.msgs.PointCloudPacked'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
    #          '@sensor_msgs/msg/CameraInfo' +
    #          '[ignition.msgs.CameraInfo'],
    #     ],
    #     remappings=[
    #         (['/world/', world,
    #           '/model/', robot_name,
    #           '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'],
    #          'oakd/rgb/preview/image_raw'),
    #         (['/world/', world,
    #           '/model/', robot_name,
    #           '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
    #          'oakd/rgb/preview/depth'),
    #         (['/world/', world,
    #           '/model/', robot_name,
    #           '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points'],
    #          'oakd/rgb/preview/depth/points'),
    #         (['/world/', world,
    #           '/model/', robot_name,
    #           '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
    #          'oakd/rgb/preview/camera_info')
    #     ]
    # )

    # Define LaunchDescription variable

    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(opfunc)

    return ld
