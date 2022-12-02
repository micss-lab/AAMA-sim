import sys
import os
import pika
import rospy
import json

from aamas_sim.src.comm_interface.rabbitmq_interface import RabbitCommunication
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rospy_message_converter import message_converter
from turtlebot3_msgs.msg import SensorState


class UWBController:

    def __init__(self, robot_count):
        rospy.init_node('uwb_controller')

        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'

        self.rabbitmq_interface = RabbitCommunication()
        self.uwb_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.robot_pos_callback, queue_size=1,
                                        buff_size=1)

        self.rabbit_queue_name = 'uwb'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

    def init_rabbit_queue(self):
        queue_base_name = self.robot_base_name + '_pos'
        for i in range(self.robot_count):
            self.rabbitmq_interface.register_queue(queue_base_name % i)

    def robot_pos_callback(self, data: ModelStates):
        robot_list = []
        for i in range(self.robot_count):
            robot_name = self.robot_base_name % i
            pos_index = data.name.index(robot_name)
            pos_dict = message_converter.convert_ros_message_to_dictionary(data.pose[pos_index])
            pos_dict['robot_id'] = i
            robot_list.append(pos_dict)

        self.rabbitmq_interface.send(self.rabbit_queue_name, robot_list)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    a = UWBController(3)
    a.run()
