import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import Imu


class ImuRobotInterface(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.robot_imu_callback,
            10)

        self.imu_msg = None

    def robot_imu_callback(self, msg):
        self.imu_msg = message_to_ordereddict(msg)
        print(self.imu_msg)



if __name__ == '__main__':
    rclpy.init()
    sub = ImuRobotInterface()
    rclpy.spin(sub)
