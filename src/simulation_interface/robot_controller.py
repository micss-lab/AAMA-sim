import sys
import os
import pika
import rospy
import json

from aamas_sim.src.comm_interface.rabbitmq_interface import RabbitCommunication
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


class RobotController:

    def __init__(self, robot_count):
        rospy.init_node('robot_controller')

        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'
        self.robot_ros_topic_base = '/%s/cmd_vel' % self.robot_base_name

        self.rabbitmq_interface = RabbitCommunication()

        self.ros_pub_list = []

        self.rabbit_queue_name = 'robot_ctrl'
        self.rabbitmq_interface.register_to_queue(self.rabbit_queue_name, self.rabbit_callback)

        self.init_ros_pubs()

    def init_ros_pubs(self):
        for i in range(self.robot_count):
            pub = rospy.Publisher(self.robot_ros_topic_base % i, Twist, queue_size=1)
            self.ros_pub_list.append(pub)

    def rabbit_callback(self, data):
        print(" [x] Received %r" % data)
        for robot_msg in data:

            try:
                robot_id = robot_msg['robot_id']
                vel_msg = Twist()
                vel_msg.linear.x = robot_msg['position']['x']
                vel_msg.linear.y = robot_msg['position']['y']
                vel_msg.linear.z = robot_msg['position']['z']
                vel_msg.angular.x = robot_msg['orientation']['x']
                vel_msg.angular.y = robot_msg['orientation']['y']
                vel_msg.angular.z = robot_msg['orientation']['z']

                self.ros_pub_list[int(robot_id)].publish(vel_msg)
            except:
                print('Invalid control message!')

    def run(self):
        self.rabbitmq_interface.start_listening()


if __name__ == '__main__':
    a = RobotController(3)
    a.run()
