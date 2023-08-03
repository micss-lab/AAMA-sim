import rospy

from aama_sim.msg import LogicalImage
from rospy_message_converter import message_converter


class LogicalCameraRobotInterface:

    def __init__(self, robot_name):
        self.robot_name = robot_name

        self.sub = rospy.Subscriber('/%s/logical_camera_image' % self.robot_name,
                                    LogicalImage,
                                    self.robot_logical_camera_callback,
                                    queue_size=1)

        self.logical_camera_msg = None

    def robot_logical_camera_callback(self, data):
        self.logical_camera_msg = message_converter.convert_ros_message_to_dictionary(data)
        self.logical_camera_msg['robot_name'] = self.robot_name
