#!/home/logesh/robotic_toolbox_ws/toolbox_env/bin/python3

"""
This is ros2 node that listeners to "/aruco_center" - center of aruco detected in another node.
The script computes the pixel error and computes the error to perfor visual servoing over it.
"""

"""
NOTE: to compute end effector velocity from pixel error - i am not using servoing based method - instead simply using pixel error as end effector vel.
TODO -> compute cam_vel from pixel error (or pixel shift/vel) - transform that to ee_vel using hand eye calibration.
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
from std_msgs.msg import Int64MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','192.168.1.9'),
                        ('robot_name','lr_mate_200id')] # custom, default
        )

        # robot model
        self.fanuc_model = Fanuc()
        self.get_logger().info(f"Fanuc model (RTB model): {self.fanuc_model.name}")

        self.aruco_center = None
        self.frame_center = np.array([320, 240])      # (640x480) - image frame
        self.pixel_error = None
        
        self.ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tracking_pose = [60.0, 300.0, 140.0, 179.0, 0.0015575372381135821, 0.0010508003178983927]
        self.tracking_pose_1 = [195.0, 220.0, 120.0, -95.0, -72.0, -37.0]
        self.triggered = False
        inc_timer_period = 1/100     # 100hz
        self.mut_cb_group = MutuallyExclusiveCallbackGroup()
        
        # PID control (TODO: tune this)
        self.KPX = 2*(0.0001)
        self.KIX = 1*(0.000001)
        self.KDX = 0*(0.00001)
        self.KPY = 2*(0.0001)
        self.KIY = 1*(0.000001)
        self.KDY = 0*(0.00001)

        # real hardware robot arm
        self.bot = robot(self.get_parameter('robot_ip').value)
        
        # ros2 communication variables
        self.aruco_center_sub = self.create_subscription(Int64MultiArray, "/aruco_center", self.aruco_center_cb, 10)
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_tracking', self.trigger_tracking_cb)
        self.ee_vel_timer = self.create_timer(inc_timer_period, self.timer_callback_3, self.mut_cb_group)


    def aruco_center_cb(self, msg):
        if (msg.data is not None):
            self.aruco_center = np.array(msg.data)
            if (self.aruco_center[0] == -1 and self.aruco_center[1] == -1):
                self.aruco_center = None

    def trigger_tracking_cb(self, request, response):
        if (request.data):
            self.triggered = True
        else:
            self.triggered = False
        
        # go to tracking position
        self.bot.write_cartesian_position(coords=self.tracking_pose, blocking=False)
        time.sleep(2)
        while (self.bot.is_moving()):
            time.sleep(0.1)

        response.success = True
        response.message = "trigger successful"
        return response

    def compute_ee_vel_from_pixel(self):
        if (self.aruco_center is not None):
            self.pixel_error = np.subtract(self.frame_center, self.aruco_center)
            self.ee_vel[0] = (self.pixel_error[0] * self.KPX) + (self.pixel_error[0] * self.KIX) + (self.pixel_error[0] * self.KDX)
            self.ee_vel[1] = (self.pixel_error[1] * self.KPY) + (self.pixel_error[1] * self.KIY) + (self.pixel_error[1] * self.KDY)
            self.ee_vel[0] *= -1    # invert x-axis
            # self.ee_vel[1] *= -1    # invert y-axis
            self.ee_vel[0], self.ee_vel[1] = self.ee_vel[1], self.ee_vel[0]
        else:
            self.pixel_error = None
            self.ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    def timer_callback_3(self):
        if (self.triggered):

            # compute pixel error and end effect velocity
            self.compute_ee_vel_from_pixel()

            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()
            # current joint position (deg to rad) + J23 coupling
            rad_arr = np.deg2rad(cur_joint_pose)
            # remove coupling - J[3]' = J[3] + J[2]
            rad_arr[2] = rad_arr[2] + rad_arr[1]

            # ee velocity to joint velocity
            current_jacobian = self.fanuc_model.jacobe(q=np.array(rad_arr))             # 6x6 matrix
            joint_vels = (np.linalg.pinv(current_jacobian) @ np.array([self.ee_vel]).T)  # 6x6 @ 6x1 => 6x1
            joint_vels = joint_vels.flatten()      # [Vj1, Vj2, Vj3, Vj4, Vj5, Vj6]

            # some filtering has to be done on the joint velocities before adding to the current joint positioni
            ### TODO: filter to joint_vels
            
            # worked after inverting the target velocity of joint 2 (may be it is inverted)
            joint_vels[1] *= -1

            # removing velocity on J4 - safety reasons
            joint_vels[3] = 0.0

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

    try:
        tracker_node = ArucoTracker()
        rclpy.spin(tracker_node)
    except Exception as e:
        print(e)
        print(f"Shutting down the node!\n")
        tracker_node.destroy_node()
        # rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    