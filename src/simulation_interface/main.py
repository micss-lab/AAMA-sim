import sys
import os
import pika
import rospy
import json
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


def callback(ch, method, properties, body):
    msg = json.loads(body)
    print(" [x] Received %r" % msg)
    vel_msg = Twist()
    vel_msg.linear.x = msg['x']
    vel_msg.linear.y = msg['y']
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    velocity_publisher.publish(vel_msg)


def rabbitmq_init():
    credentials = pika.PlainCredentials('dogu', 'dogu')
    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host='localhost', port=5672, credentials=credentials))
    channel = connection.channel()

    channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)

    print(' [*] Waiting for messages. To exit press CTRL+C')
    channel.start_consuming()


if __name__ == '__main__':
    try:
        rospy.init_node('robot_controller')

        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rabbitmq_init()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
