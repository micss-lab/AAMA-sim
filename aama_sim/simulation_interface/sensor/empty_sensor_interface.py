import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu

try:
    from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
except:
    from build.aama_sim.aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication


class EmptyInterface(Node):
    """
    This is an empty interface that can be used as a template for creating new interfaces.
    """

    def __init__(self):
        super().__init__('empty_interface')

        # This is how you can get the `robot_count` parameter from the launch file
        self.declare_parameter('robot_count', 1)
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        # This is how you can create a RabbitMQ interface
        self.rabbitmq_interface = RabbitCommunication()
        self.robot_sensors = []

        # Define the name of the RabbitMQ queue for the sensor
        self.rabbit_queue_name = '<sensor_rabbitmq_queue_name>'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        """
        This is how you can create a subscription to a ROS2 topic.
        Make sure the sensor is publishing to a ROS2 topic.
        """
        self.subscription = self.create_subscription(
            Imu,  # Change this to the message type that the sensor is publishing
            'imu',  # Change this to the name of the ROS2 topic that the sensor is publishing to
            self.imu_callback,  # Change this to the callback function that will be called when a message is received
            10)

        self.imu_msg = dict()

    def imu_callback(self, msg: Imu):
        """
        This is the callback function that will be called when a message is received from the sensor.
        :param msg:
        :return:
        """
        msg.header.frame_id = msg.header.frame_id.split('/')[0]
        self.imu_msg[msg.header.frame_id] = message_to_ordereddict(msg)

        self.rabbitmq_interface.send(self.rabbit_queue_name, list(self.imu_msg.values()))


def main(args=None):
    """
    This is the main function that will be called when the node is executed.
    """
    rclpy.init(args=args)
    empty_interface = EmptyInterface()
    rclpy.spin(empty_interface)
    empty_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
