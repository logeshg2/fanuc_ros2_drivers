#!/usr/bin/env python3
import sys
import os
import rclpy
import numpy as np
import time

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver

from ComDependencies.robot_controller import robot
from fanuc_interfaces.msg import CurCartesian
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class current_cartesian(Node):
    def __init__(self):
        super().__init__('cur_cart')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.inc_triggered = False
        self.mut_cb_group = MutuallyExclusiveCallbackGroup()

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_publisher(CurCartesian, f"{self.get_parameter('robot_name').value}/cur_cartesian", 10)
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_inc_movement', self.trigger_inc_move_cb)
        timer_period = 0.5
        inc_timer_period = 1/100     # 100hz
        self.timer = self.create_timer(timer_period, self.timer_callback, self.mut_cb_group)
        self.inc_timer = self.create_timer(inc_timer_period, self.timer_callback_2, self.mut_cb_group)

    def trigger_inc_move_cb(self, request, response):
        if (request.data):
            self.inc_triggered = True
        else:
            self.inc_triggered = False
        
        response.success = True
        response.message = "trigger successful"
        return response
    
    def timer_callback_2(self):
        # movement should not take place
        if (self.inc_triggered): # and (not self.bot.is_moving())):
            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()      # [X, Y, Z, W, P, R]
            # increment by small distance (0.1)
            kp = 0.01
            kd = 0.001
            ki = 0.000
            inc = 50.0
            inc_t = (inc * kp) + (inc * kd) + (inc * ki)
            target_joint_pose = (np.array(cur_joint_pose) + np.array([inc_t, inc_t, -inc_t, 0.0, 0.0, 0.0])).tolist()       # the 2nd componenet can be velocity integral (my hypothesis)
            # call robot 
            self.get_logger().info(f"{target_joint_pose}")

            target_joint_pose = [0.0, 0.0] + target_joint_pose + [0.0, 0.0, 0.0]
            s = time.perf_counter()
            # self.bot.write_joint_pose(target_joint_pose, blocking=False)
            FANUCethernetipDriver.writeJointPositionRegister(self.bot.robot_IP, self.bot.PRNumber, target_joint_pose)
            FANUCethernetipDriver.writeR_Register(self.bot.robot_IP, self.bot.start_register, 1)
            self.get_logger().info(f"Time taken: {time.perf_counter() - s}")

    def timer_callback(self):
        msg = CurCartesian()                                          
        msg.pose = self.bot.read_current_cartesian_pose()                                  
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.pose)


def main(args=None):
    rclpy.init(args=args)

    publisher = current_cartesian()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
