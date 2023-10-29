import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
except:
    from build.aama_sim.aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('robot_count')

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.robot_base_name = 'aama_robot_%s'
        self.robot_ros_topic_base = '/model/%s/cmd_vel' % self.robot_base_name

        self.rabbitmq_interface = RabbitCommunication()

        self.ros_pub_list = []

        self.rabbit_queue_name = 'robot_ctrl'
        self.rabbitmq_interface.register_to_queue(self.rabbit_queue_name, self.rabbit_callback)

        self.init_ros_pubs()

    def init_ros_pubs(self):
        for i in range(self.robot_count):
            pub = self.create_publisher(
                Twist,
                self.robot_ros_topic_base % i,
                10
            )
            self.ros_pub_list.append(pub)

    def rabbit_callback(self, data):
        print(" [x] Received %r" % data)
        for robot_msg in data:
            try:
                robot_id = robot_msg['robot_id']
                vel_msg = Twist()
                vel_msg.linear.x = robot_msg['position']['x']
                vel_msg.linear.y = robot_msg['position']['y']
                vel_msg.linear.z = robot_msg['position']['z']
                vel_msg.angular.x = robot_msg['orientation']['x']
                vel_msg.angular.y = robot_msg['orientation']['y']
                vel_msg.angular.z = robot_msg['orientation']['z']

                self.ros_pub_list[int(robot_id)].publish(vel_msg)
            except:
                print('Invalid control message!')

    def run(self):
        self.rabbitmq_interface.start_listening()


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    robot_controller.run()
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
