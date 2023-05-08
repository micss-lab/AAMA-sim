import rospy

from sensor_msgs.msg import Imu
from rospy_message_converter import message_converter


class ImuRobotInterface:

    def __init__(self, robot_name):
        self.robot_name = robot_name

        self.sub = rospy.Subscriber('/%s/imu' % self.robot_name, Imu, self.robot_imu_callback, queue_size=1)

        self.imu_msg = None

    def robot_imu_callback(self, data):
        data.header.frame_id = self.robot_name
        self.imu_msg = message_converter.convert_ros_message_to_dictionary(data)
