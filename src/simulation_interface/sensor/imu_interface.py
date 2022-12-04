import rospy
from aamas_sim.src.comm_interface.rabbitmq_interface import RabbitCommunication
from aamas_sim.src.simulation_interface.sensor.imu_robot_interface import ImuRobotInterface
from sensor_msgs.msg import Imu


class ImuInterface:

    def __init__(self, robot_count):
        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'

        self.rabbitmq_interface = RabbitCommunication()
        self.robot_imus = []

        self.rabbit_queue_name = 'imu'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        self.init_robot_imu_subs()

    def init_robot_imu_subs(self):
        for i in range(self.robot_count):
            robot_imu = ImuRobotInterface(self.robot_base_name % i)
            self.robot_imus.append(robot_imu)

    def send_imu_packet(self):
        imu_packet = []
        for robot_imu in self.robot_imus:
            imu_packet.append(robot_imu.imu_msg)

        self.rabbitmq_interface.send(self.rabbit_queue_name, imu_packet)

    def dry_run(self):
        while not rospy.is_shutdown():
            self.send_imu_packet()


if __name__ == '__main__':
    rospy.init_node('test_imu')
    a = ImuInterface(3)
    a.dry_run()
