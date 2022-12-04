import rospy
from aamas_sim.comm_interface.rabbitmq_interface import RabbitCommunication
from aamas_sim.simulation_interface.sensor.sonar_robot_interface import SonarRobotInterface


class SonarInterface:

    def __init__(self, robot_count):
        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'

        self.rabbitmq_interface = RabbitCommunication()
        self.robot_sonars = []

        self.rabbit_queue_name = 'sonar'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        self.init_robot_sonar_subs()

    def init_robot_sonar_subs(self):
        for i in range(self.robot_count):
            robot_sonar = SonarRobotInterface(self.robot_base_name % i)
            self.robot_sonars.append(robot_sonar)

    def send_sonar_packet(self):
        sonar_packet = []
        for robot_sonar in self.robot_sonars:
            sonar_packet.append(robot_sonar.sonar_msg)

        self.rabbitmq_interface.send(self.rabbit_queue_name, sonar_packet)

    def dry_run(self):
        while not rospy.is_shutdown():
            self.send_sonar_packet()


# if __name__ == '__main__':
#     rospy.init_node('test_imu')
#     a = SonarInterface(3)
#     a.dry_run()
