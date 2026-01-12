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
from fanuc_interfaces.srv import EETwist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class EETwistCMD(Node):
    def __init__(self):
        super().__init__('cur_cart')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','192.168.1.9'),
                        ('robot_name','lr_mate_200id')] # custom, default
        )

        # robot model
        self.fanuc_model = Fanuc()
        self.get_logger().info(f"Fanuc model (RTB model): {self.fanuc_model.name}")

        self.ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.inc_triggered = False
        inc_timer_period = 1/100     # 100hz
        self.mut_cb_group = MutuallyExclusiveCallbackGroup()

        # real hardware robot arm
        self.bot = robot(self.get_parameter('robot_ip').value)
        
        self.twist_sub = self.create_subscription(TwistStamped, '/fanuc_servo/delta_twist_cmds', self.twist_callback, 10)
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_inc_movement', self.trigger_inc_move_cb)
        self.ee_twist_srv = self.create_service(EETwist, '/fanuc_ee_twist', self.ee_twist_cb)
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
        # check the latest msg
        cur_time_ns = self.get_clock().now().nanoseconds
        msg_time_ns = msg.header.stamp.sec * (10 ** 9) + msg.header.stamp.nanosec
        time_diff_ms = (cur_time_ns - msg_time_ns) // (10 ** 6)     # millisecond

        if (abs(time_diff_ms) < 200):       # 200 ms
            self.ee_vel = [msg.twist.linear.x, 
                           msg.twist.linear.y, 
                           msg.twist.linear.z, 
                           msg.twist.angular.x, 
                           msg.twist.angular.y, 
                           msg.twist.angular.z
                           ]

    def ee_twist_cb(self, request, response):
        self.ee_vel = [request.x_dot,
                       request.y_dot,
                       request.z_dot,
                       request.w_dot,
                       request.p_dot,
                       request.r_dot]
        
        self.get_logger().info(f"Target EE Vel received: {self.ee_vel}")
        response.success = True
        response.message = f"Target ee velocity set"
        return response

    def timer_callback_3(self):
        if (self.inc_triggered or np.any(np.array(self.ee_vel))):
            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()
            # current joint position (deg to rad) + J23 coupling
            rad_arr = np.deg2rad(cur_joint_pose)
            # remove coupling - J[3]' = J[3] + J[2]
            rad_arr[2] = rad_arr[2] + rad_arr[1]
            print(rad_arr)

            # ee velocity to joint velocity (for current joint angles)
            current_jacobian = self.fanuc_model.jacobe(q=np.array(rad_arr))             # 6x6 matrix
            joint_vels = (np.linalg.pinv(current_jacobian) @ np.array([self.ee_vel]).T)  # 6x6 @ 6x1 => 6x1
            joint_vels = joint_vels.flatten()      # [Vj1, Vj2, Vj3, Vj4, Vj5, Vj6]

            # some filtering has to be done on the joint velocities before adding to the current joint positioni
            ### TODO: filter to joint_vels (or some PID control) - not sure
            
            # worked after inverting the target velocity of joint 2 (may be it is inverted)
            joint_vels[1] *= -1

            # add that to current joint position
            target_rad_arr = np.add(rad_arr, joint_vels)
            
            # adding coupling - J[3]' = J[3] - J[2]
            target_rad_arr[2] = target_rad_arr[2] - target_rad_arr[1]
            target_joint_pose = np.rad2deg(target_rad_arr).tolist()

            self.get_logger().info(f"target ee vel: {self.ee_vel}")

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
    