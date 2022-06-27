'''
Read odom data from ROS nodes and save it to csv files.
@Author: AVANT UFMG
'''

import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from geometry_msgs.msg import Vector3
from rosidl_runtime_py import message_to_csv


class OdomWrapper(Node):
    def __init__(self, data_file_path: str, cleanup: bool = True) -> None:
        '''
        :param data_file_path: path to save csv file.
        :param cleanup: delete previous saved files if they exist.
        '''

        super().__init__('odom_wrapper')

        self.data_file_path = data_file_path
        self.pose_data = list()

        # odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        odom_sub = self.create_subscription(Pose, 'turtle1/pose', self.odom_cb, 10)

        if cleanup:
            self.cleanup_data()
            
    def odom_cb(self, msg: Pose):
        msg2 = Vector3(x=msg.x, y=msg.y, z=msg.theta)
        with open(self.data_file_path, mode='a') as file:
            csv_data = message_to_csv(msg2)
            file.write(csv_data + '\n')

    def cleanup_data(self):
        if os.path.exists(self.data_file_path):
            os.remove(self.data_file_path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomWrapper('data/odom.csv')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        