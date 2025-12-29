#!/usr/bin/env python3
import sys
import os
import rclpy

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver

from ComDependencies.robot_controller import robot
from fanuc_interfaces.msg import ForceSensor
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class check_movement(Node):
    def __init__(self):
        super().__init__('move_pub')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_publisher(ForceSensor, f"{self.get_parameter('robot_name').value}/force_sensor", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # these are mean force sensor values
        cur_fs_values = self.bot.read_force_sensor_values()     # [Fz, Mx, My]

        msg = ForceSensor()
        print("\n\n",cur_fs_values, "\n\n")
        msg.fz = float(cur_fs_values[0])
        msg.mx = float(cur_fs_values[1])
        msg.my = float(cur_fs_values[2])

        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing (FS): ', msg.fz, msg.mx, msg.my)


def main(args=None):
    rclpy.init(args=args)

    publisher = check_movement()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
