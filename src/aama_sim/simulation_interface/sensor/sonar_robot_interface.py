import rospy
import math

from sensor_msgs.msg import Range
from rospy_message_converter import message_converter


class SonarRobotInterface:

    def __init__(self, robot_name):
        self.robot_name = robot_name

        self.sub = rospy.Subscriber('/%s/sonar' % self.robot_name, Range, self.robot_sonar_callback, queue_size=1,
                                    buff_size=1)

        self.sonar_msg = None

    def robot_sonar_callback(self, data: Range):
        data.header.frame_id = self.robot_name
        data.min_range = round(data.min_range, 3)
        data.range = round(data.range, 3)
        data.field_of_view = round(math.degrees(data.field_of_view), 3)
        self.sonar_msg = message_converter.convert_ros_message_to_dictionary(data)
