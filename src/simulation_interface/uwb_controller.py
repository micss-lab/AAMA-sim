import sys
import os
import pika
import rospy
import json
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rospy_message_converter import message_converter
from turtlebot3_msgs.msg import SensorState


class UWBController:

    def __init__(self, robot_id):
        rospy.init_node('uwb_controller_%s' % robot_id)

        self.robot_id = robot_id
        self.rabbit_channel = None
        self.rabbit_queue_name = 'tb3_%s_uwb' % self.robot_id

        self.connect_to_rabbitMQ()
        self.uwb_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.robot_pos_callback, queue_size=1,
                                        buff_size=1)

    def connect_to_rabbitMQ(self):
        credentials = pika.PlainCredentials('dogu', 'dogu')
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(host='localhost', port=5672, credentials=credentials))

        self.rabbit_channel = connection.channel()

        self.rabbit_channel.queue_declare(queue=self.rabbit_queue_name,
                                          arguments={"x-max-length": 1,
                                                     "x-overflow": "drop-head"})

    def robot_pos_callback(self, data: ModelStates):
        robot_name = 'tb3_%s' % self.robot_id
        pos_index = data.name.index(robot_name)
        pos_dict = message_converter.convert_ros_message_to_dictionary(data.pose[pos_index])

        self.rabbit_channel.basic_publish(exchange='', routing_key=self.rabbit_queue_name, body=json.dumps(pos_dict))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    a = UWBController(0)
    a.run()
