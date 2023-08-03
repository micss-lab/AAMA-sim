import rospy
from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
from aama_sim.simulation_interface.sensor.logical_camera_robot_interface import LogicalCameraRobotInterface
from sensor_msgs.msg import Imu


class LogicalCameraInterface:

    def __init__(self, robot_count):
        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'

        self.rabbitmq_interface = RabbitCommunication()
        self.robot_logical_cameras = []

        self.rabbit_queue_name = 'logical_camera'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        self.init_robot_logical_camera_subs()

    def init_robot_logical_camera_subs(self):
        for i in range(self.robot_count):
            robot_logical_camera = LogicalCameraRobotInterface(self.robot_base_name % i)
            self.robot_logical_cameras.append(robot_logical_camera)

    def send_logical_camera_packet(self):
        logical_camera_packet = []
        for logical_camera in self.robot_logical_cameras:
            logical_camera_packet.append(logical_camera.logical_camera_msg)

        self.rabbitmq_interface.send(self.rabbit_queue_name, logical_camera_packet)

    def dry_run(self):
        while not rospy.is_shutdown():
            self.send_logical_camera_packet()


# if __name__ == '__main__':
#     rospy.init_node('test_imu')
#     a = LogicalCameraInterface(3)
#     a.dry_run()
