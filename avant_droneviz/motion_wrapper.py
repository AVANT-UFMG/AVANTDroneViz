#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import csv
from turtlesim.msg import Pose
import datetime


class SubscriberClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')

        self.fieldnames = ['velocidade_linear', 'velocidade_angular', 'acceleração_linear','acceleração_angular', 'tempo']
        with open('velocidade.csv', 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()

        self.v_lin = 0
        self.v_ang = 0
        self.t = datetime.datetime.now()
        
        self.subscription = self.create_subscription(Pose,'turtle1/pose',self.subscriber_callback,10)
        self.subscription 



    def subscriber_callback(self, msg): 

        self.get_logger().info('Recieved - Linear Velocity: %f, Angualr Velocity: %f' % (msg.linear_velocity, msg.angular_velocity))
        
        now = datetime.datetime.now()

        time_diff = time_difference(self.t, now)

        accel_lin = find_accel(time_diff, msg.linear_velocity, self.v_lin)
        accel_ang = find_accel(time_diff, msg.angular_velocity, self.v_ang)

        with open('velocidade.csv', 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames= self.fieldnames)
            info = {'velocidade_linear': msg.linear_velocity, 'velocidade_angular': msg.angular_velocity, 'acceleração_linear': accel_lin, 'acceleração_angular': accel_ang, 'tempo': now.strftime("%H:%M:%S")}
            csv_writer.writerow(info)
            csv_file.close()

        self.v_lin = msg.linear_velocity
        self.v_ang = msg.angular_velocity
        self.t = now

        
def time_difference(t1, t2):
    h = t2.hour
    m = t2.minute
    s = t2.second
    while(t1.second > s):
        m -= 1
        s +=60
    seconds = s - t1.second
    while(t1.minute > m):
        h -= 1
        m += 60
    minuites = m - t1.minute
    hours = h - t1.hour
    return {"hour":hours,"minute": minuites,"second": seconds}

def find_accel(time_diff, v1, v2):
    if time_diff["second"] == 0 and v1 != v2:
        accel = 9999
    elif v1 == v2:
        accel = 0
    elif time_diff["second"] != 0 and v1 != v2:
        accel = (v2- v1)/(time_diff["second"])
    return accel

def main():

    rclpy.init()
    node = SubscriberClass()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
