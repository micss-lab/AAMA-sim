import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu

try:
    from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
except:
    from build.aama_sim.aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication


class ImuInterface(Node):

    def __init__(self):
        super().__init__('imu_interface')

        self.declare_parameter('robot_count', ParameterDescriptor(description='Robots in the simulation'))

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        self.rabbitmq_interface = RabbitCommunication()
        self.robot_imus = []

        self.rabbit_queue_name = 'imu'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)

        self.imu_msg = dict()

    def imu_callback(self, msg: Imu):
        msg.header.frame_id = msg.header.frame_id.split('/')[0]
        self.imu_msg[msg.header.frame_id] = message_to_ordereddict(msg)

        self.rabbitmq_interface.send(self.rabbit_queue_name, list(self.imu_msg.values()))


def main(args=None):
    rclpy.init(args=args)
    imu_interface = ImuInterface()
    rclpy.spin(imu_interface)
    imu_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
