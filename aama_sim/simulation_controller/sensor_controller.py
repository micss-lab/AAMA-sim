#!/user/bin/env python3

import rospy
from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
from aama_sim.simulation_interface.sensor.imu_interface import ImuInterface
from aama_sim.simulation_interface.sensor.sonar_interface import SonarInterface


class SensorController:

    def __init__(self):
        rospy.init_node('sensor_controller')

        self.robot_base_name = 'tb3_%s'  # Replace with rosparam
        self.robot_count = 3  # Replace with rosparam

        self.rabbitmq_comm = RabbitCommunication()

        self.imu_interface = ImuInterface(self.robot_count)
        self.sonar_interface = SonarInterface(self.robot_count)

        self.rate = rospy.Rate(30)

    def fetch_sensor_msgs(self):
        self.imu_interface.send_imu_packet()
        self.sonar_interface.send_sonar_packet()

    def run(self):
        while not rospy.is_shutdown():
            self.fetch_sensor_msgs()
            self.rate.sleep()


if __name__ == '__main__':
    a = SensorController()
    a.run()
