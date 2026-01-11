#!/home/logesh/robotic_toolbox_ws/toolbox_env/bin/python3

"""
This is ros2 node that listeners to '/fanuc_servo/delta_twist_cmds' twist message (EE velocity).
Uses rtb to compute joint velocities from ee velocity and cmds the arm to move the joint in that velocity.
"""

import sys
import time
import rclpy
import numpy as np

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver
from ComDependencies.robot_controller import robot

from fanuc_model import Fanuc

from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class EETwistCMD(Node):
    def __init__(self):
        super().__init__('cur_cart')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        # robot model
        self.fanuc_model = Fanuc()
        self.get_logger().info(f"Fanuc model (RTB model): {self.fanuc_model.name}")

        self.ee_vel = None
        self.inc_triggered = False
        inc_timer_period = 1/100     # 100hz
        self.mut_cb_group = MutuallyExclusiveCallbackGroup()

        # real hardware robot arm
        self.bot = robot(self.get_parameter('robot_ip').value)
        
        self.twist_sub = self.create_subscription(TwistStamped, '/fanuc_servo/delta_twist_cmds', self.twist_callback, 10)
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_inc_movement', self.trigger_inc_move_cb)
        # self.inc_timer = self.create_timer(inc_timer_period, self.timer_callback_2, self.mut_cb_group)
        self.ee_vel_timer = self.create_timer(inc_timer_period, self.timer_callback_3, self.mut_cb_group)

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

    def twist_callback(self, msg):
        # take the latest message - WIP (this might also go wrong)
        if (abs(self.get_clock().now() - msg.header.stamp.from_msg()) < 200):       # 200 ms i guess
            self.ee_vel = [msg.twist.linear.x, 
                           msg.twist.linear.y, 
                           msg.twist.linear.z, 
                           msg.twist.angular.x, 
                           msg.twist.angular.y, 
                           msg.twist.angular.z
                           ]

    def timer_callback_3(self):
        if (self.inc_triggered):
            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()      # [X, Y, Z, W, P, R]
            # current joint position (deg to rad) + J23 coupling
            rad_arr = np.deg2rad(cur_joint_pose)
            # remove coupling - J[3]' = J[3] + J[2]
            rad_arr[2] = rad_arr[2] + rad_arr[1]

            # ee velocity to joint velocity (for current joint angles)
            current_jacobian = self.fanuc_model.jacobe(q=np.array(rad_arr))             # 6x6 matrix
            joint_vels = (np.linalg.pinv(current_jacobian) @ np.array([self.ee_vel]).T)  # 6x6 @ 6x1 => 6x1
            joint_vels = joint_vels.flatten().tolist()      # [Vj1, Vj2, Vj3, Vj4, Vj5, Vj6]

            # some filtering has to be done on the joint velocities before adding to the current joint positioni
            ### TODO: filter to joint_vels (or some PID control) - not sure

            # add that to current joint position to create target joint position
            target_joint_pose = cur_joint_pose
            target_joint_pose[0] += joint_vels[0]
            target_joint_pose[1] += joint_vels[1]
            target_joint_pose[2] += joint_vels[2]
            target_joint_pose[3] += joint_vels[3]
            target_joint_pose[4] += joint_vels[4]
            target_joint_pose[5] += joint_vels[5]

            # write register and sync-movement
            self.get_logger().info(f"Computed Joint Position: {target_joint_pose}")
            self.bot.write_joint_pose(target_joint_pose, blocking=False)


def main(args=None):
    rclpy.init(args=args)

    ee_twist_node = EETwistCMD()

    rclpy.spin(ee_twist_node)

    ee_twist_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    