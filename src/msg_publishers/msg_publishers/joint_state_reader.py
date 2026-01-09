#!/usr/bin/env python3

"""
This particular node is used for moving the real robot - read the '/joint_states' topic message and perform movement action.
NOTE: "Joint State Reader" - Reads rviz plans robot joint position and performs actions through direct writing registers (no ros2 actions involved).
"""

import sys
import os
import rclpy
import numpy as np
from collections import deque

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver

from ComDependencies.robot_controller import robot
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class joint_state_reader(Node):
    def __init__(self):
        super().__init__('joint_reader_2_real')

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
        self.temp_joint_name = None
        self.temp_joint_val = None
        self.joint_queue = deque()
        self.prev_joint_val = None
        self.joint_angle_tolerance = 0.01   # ~ 1 deg

        self.ret_cb_group = ReentrantCallbackGroup()

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_subscription(JointState, "joint_states", self.joint_states_cb, 10, callback_group=self.ret_cb_group)
        timer_period = 1/100     # 20hz
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.move_arm_timer = self.create_timer(timer_period, self.timer_callback_2,  callback_group=self.ret_cb_group)

    def joint_states_cb(self, msg):
        # put in queue only if there is significant change in target joint position
        # self.get_logger().info(f"{(np.array(self.prev_joint_val) - np.array(msg.position))}")
        if (self.prev_joint_val is None):
            # NOTE: the joint name comming in may not be in order (so adjusting the order)
            self.temp_joint_name = msg.name
            self.temp_joint_val = msg.position
            target_joint_val = self.correctJointOrder() # IMP

            self.prev_joint_val = target_joint_val
            self.joint_queue.append(target_joint_val)
        elif (np.any((np.array(self.prev_joint_val) - np.array(self.correctJointOrder(msg.name, msg.position))) > self.joint_angle_tolerance)):
            # NOTE: the joint name comming in may not be in order (so adjusting the order)
            self.get_logger().info("here")
            self.temp_joint_name = msg.name
            self.temp_joint_val = msg.position
            target_joint_val = self.correctJointOrder() # IMP

            self.prev_joint_val = target_joint_val
            self.joint_queue.append(target_joint_val)

    def timer_callback_2(self):
        if (self.joint_queue):
            # first check if robot is free or not moving to pass the move robot command
            if (not self.bot.is_moving()):
                target_joint_pos = self.joint_queue.popleft()

                # handle J23 coupling factor (for lr_mate_200id4s - J3' = J3 - J2) - (IMP)
                target_joint_pos[2] = target_joint_pos[2] - target_joint_pos[1]
                # radian to deg converstion
                target_joint_pos = np.rad2deg(target_joint_pos)
                self.get_logger().info(f"Deque Size: {len(self.joint_queue)}")
                # self.get_logger().info(f"Target joint position: {np.round(target_joint_pos, 3)}")
                
                self.bot.start_cnt_motion()
                self.bot.write_joint_pose(joint_position_array=target_joint_pos, blocking=False)
            else:
                self.get_logger().info(f"Robot is moving waiting for arm to stop")
        else:
            # self.get_logger().info(f"Joint queue is empty")
            pass

    
    # TODO - need to modify the poorly construced function
    def correctJointOrder(self, joint_names = [], joint_values = []):
        if (joint_names != [] and joint_values != []):
            self.temp_joint_name = joint_names
            self.temp_joint_val = joint_values
        ret_joint_val = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for idx, jt_name in enumerate(self.temp_joint_name):
            jt_num = int(jt_name[-1])
            ret_joint_val[jt_num - 1] = self.temp_joint_val[idx]       # 0 - indexing

        # TODO - hand error properly
        return ret_joint_val


    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()       
        msg.name = self.joint_names 

        # degree to radian
        deg_arr = self.bot.read_current_joint_position()
        rad_arr = list(np.deg2rad(deg_arr)) 

        # handle J23 coupling factor (for lr_mate_200id4s - J3' = J3 + J2) - (IMP)
        rad_arr[2] = rad_arr[2] + rad_arr[1]

        msg.position = rad_arr
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.position)


def main(args=None):
    rclpy.init(args=args)

    publisher = joint_state_reader()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
