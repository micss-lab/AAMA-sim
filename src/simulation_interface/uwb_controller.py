import math
import rospy

from aamas_sim.src.comm_interface.rabbitmq_interface import RabbitCommunication
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from rospy_message_converter import message_converter
from turtlebot3_msgs.msg import SensorState


def convert_quaternion_orientation_to_euler(ori_dict: dict):
    orientation_list = [ori_dict['x'], ori_dict['y'], ori_dict['z'], ori_dict['w']]
    return euler_from_quaternion(orientation_list)


class UWBController:

    def __init__(self, robot_count):
        self.robot_count = robot_count
        self.robot_base_name = 'tb3_%s'

        self.rabbitmq_interface = RabbitCommunication()
        self.uwb_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.robot_pos_callback, queue_size=1,
                                        buff_size=1)

        self.rabbit_queue_name = 'uwb'
        self.rabbitmq_interface.register_queue(self.rabbit_queue_name)

    def init_rabbit_queue(self):
        queue_base_name = self.robot_base_name + '_pos'
        for i in range(self.robot_count):
            self.rabbitmq_interface.register_queue(queue_base_name % i)

    def robot_pos_callback(self, data: ModelStates):
        robot_list = []
        for i in range(self.robot_count):
            robot_name = self.robot_base_name % i
            pos_index = data.name.index(robot_name)
            pos_dict = message_converter.convert_ros_message_to_dictionary(data.pose[pos_index])

            refined_pos_dict = self.refine_uwb_packet(pos_dict, i)
            robot_list.append(refined_pos_dict)

        self.rabbitmq_interface.send(self.rabbit_queue_name, robot_list)

    def refine_uwb_packet(self, pos_dict, robot_id):
        pos_dict['robot_id'] = robot_id

        orientation_quaternion = pos_dict['orientation']
        orientation_euler = convert_quaternion_orientation_to_euler(orientation_quaternion)
        pos_dict['orientation'] = {
            'x': round(math.degrees(orientation_euler[0]), 3),  # roll
            'y': round(math.degrees(orientation_euler[1]), 3),  # pitch
            'z': round(math.degrees(orientation_euler[2]), 3)  # yaw
        }
        pos_dict['position'] = {
            'x': round(pos_dict['position']['x'], 3),
            'y': round(pos_dict['position']['y'], 3),
            'z': round(pos_dict['position']['z'], 3),
        }

        return pos_dict

    def dry_run(self):
        rospy.spin()

# if __name__ == '__main__':
#     a = UWBController(3)
#     a.run()
