import rospy
from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
from aama_sim.simulation_interface.robot_controller import RobotController
from aama_sim.simulation_interface.uwb_controller import UWBController


class SimulationController:

    def __init__(self):
        rospy.init_node('simulation_controller')

        self.robot_base_name = 'tb3_%s'  # Replace with rosparam
        self.robot_count = 3  # Replace with rosparam

        self.rabbitmq_comm = RabbitCommunication()

        self.uwb_controller = UWBController(robot_count=self.robot_count)
        self.robot_controller = RobotController(robot_count=self.robot_count)

    def init_sensor_controllers(self, robot_id):
        pass

    def run_simulation_controller(self):
        self.robot_controller.run()


if __name__ == '__main__':
    a = SimulationController()
    a.run_simulation_controller()
