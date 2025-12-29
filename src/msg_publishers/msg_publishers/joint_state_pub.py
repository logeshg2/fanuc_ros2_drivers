#!/usr/bin/env python3

"""
This particular node is used for rviz2 simulation - reads the current joint angle and publishes as joint states for rviz2.
"""

import sys
import os
import rclpy
import numpy as np

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from sensor_msgs.msg import JointState
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class current_joint(Node):
    def __init__(self):
        super().__init__('curr_joint')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        timer_period = 1/20     # 20hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()       
        msg.name = self.joint_names 

        # degree to radian
        deg_arr = self.bot.read_current_joint_position()
        rad_arr = list(np.deg2rad(deg_arr))                          
        msg.position = rad_arr
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.position)


def main(args=None):
    rclpy.init(args=args)

    publisher = current_joint()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
