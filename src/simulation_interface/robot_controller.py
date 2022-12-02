import sys
import os
import pika
import rospy
import json
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


class RobotController:

    def __init__(self, robot_id):
        rospy.init_node('robot_controller_%s' % robot_id)

        self.robot_id = robot_id
        self.rabbit_channel = None
        self.rabbit_queue_name = 'tb3_%s_ctrl' % self.robot_id

        self.connect_to_rabbitMQ()
        self.ctl_pub = rospy.Publisher('/tb3_%s/cmd_vel' % self.robot_id, Twist, queue_size=1)

    def connect_to_rabbitMQ(self):
        credentials = pika.PlainCredentials('dogu', 'dogu')
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(host='localhost', port=5672, credentials=credentials))

        self.rabbit_channel = connection.channel()

        self.rabbit_channel.queue_declare(queue=self.rabbit_queue_name,
                                          arguments={"x-max-length": 1,
                                                     "x-overflow": "drop-head"})

    def rabbit_callback(self, ch, method, properties, body):
        msg = json.loads(body)
        print(" [x] Received %r" % msg)
        vel_msg = Twist()
        vel_msg.linear.x = msg['x']
        vel_msg.linear.y = msg['y']
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.ctl_pub.publish(vel_msg)

    def run(self):
        self.rabbit_channel.basic_consume(queue=self.rabbit_queue_name,
                                          on_message_callback=self.rabbit_callback,
                                          auto_ack=True)

        print(' [*] Waiting for messages. To exit press CTRL+C')
        self.rabbit_channel.start_consuming()

if __name__ == '__main__':
    a = RobotController(0)
    a.run()