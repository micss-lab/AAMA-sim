import math
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_msgs.msg import TFMessage

try:
    from aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication
except:
    from build.aama_sim.aama_sim.comm_interface.rabbitmq_interface import RabbitCommunication


def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion to Euler angles (yaw, pitch, and roll) in radians.
    The quaternion should be provided as four values (w, x, y, z).
    """

    # Calculate the Euler angles
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z, pitch_y, roll_x  # in radians


def convert_quaternion_orientation_to_euler(ori_dict: dict):
    return quaternion_to_euler(ori_dict['x'], ori_dict['y'], ori_dict['z'], ori_dict['w'])


class UWBController(Node):

    def __init__(self):
        super().__init__('uwb_controller')

        self.declare_parameter('robot_count', 1)
        self.declare_parameter('robot_name', "aama_robot")

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.robot_base_name = self.get_parameter('robot_name').get_parameter_value().string_value + '_%s'
        self.robot_base_topic = '/model/%s/pose_static' % self.robot_base_name

        self.rabbitmq_interface = RabbitCommunication()

        self.rabbit_queue_name = 'uwb'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

        self.robot_dict = dict()
        self.init_subscribers()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_subscribers(self):
        for i in range(self.robot_count):
            self.create_subscription(
                TFMessage,
                self.robot_base_topic % i,
                self.robot_pos_callback,
                10
            )

    def init_rabbit_queue(self):
        queue_base_name = self.robot_base_name + '_pos'
        for i in range(self.robot_count):
            self.rabbitmq_interface.register_queue(queue_base_name % i)

    def robot_pos_callback(self, msg: TFMessage):
        for tf_msg in msg.transforms:
            # if re.fullmatch(r'aama_robot_\d+', tf_msg.header.frame_id) is not None:
            if tf_msg.header.frame_id == 'visualize_lidar_world':
                pos_dict = message_to_ordereddict(tf_msg)
                refined_pos_dict = self.refine_uwb_packet(pos_dict, tf_msg.child_frame_id)
                self.robot_dict[tf_msg.child_frame_id] = refined_pos_dict

    def refine_uwb_packet(self, pos_dict, robot_id):
        refined_pos_dict = dict()
        refined_pos_dict['robot_id'] = robot_id.split('_')[-1]
        refined_pos_dict['header'] = pos_dict['header']
        refined_pos_dict['header']['frame_id'] = self.robot_base_name % refined_pos_dict['robot_id']

        orientation_quaternion = pos_dict['transform']['rotation']
        orientation_euler = convert_quaternion_orientation_to_euler(orientation_quaternion)
        refined_pos_dict['orientation'] = {
            'x': round(math.degrees(orientation_euler[0]), 3),  # roll
            'y': round(math.degrees(orientation_euler[1]), 3),  # pitch
            'z': round(math.degrees(orientation_euler[2]), 3)  # yaw
        }
        refined_pos_dict['position'] = {
            'x': round(pos_dict['transform']['translation']['x'], 3),
            'y': round(pos_dict['transform']['translation']['y'], 3),
            'z': round(pos_dict['transform']['translation']['z'], 3),
        }

        return refined_pos_dict

    def timer_callback(self):
        self.rabbitmq_interface.send(self.rabbit_queue_name, list(self.robot_dict.values()))


def main(args=None):
    rclpy.init(args=args)
    uwb_controller = UWBController()
    rclpy.spin(uwb_controller)
    uwb_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
