#!/home/logesh/robotic_toolbox_ws/toolbox_env/bin/python3

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


class Move2PoseCMD(Node):
    def __init__(self):
        super().__init__('move2pose_node')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','192.168.1.9'),
                        ('robot_name','lr_mate_200id')] # custom, default
        )


        # robot model
        self.fanuc_model = Fanuc()
        self.get_logger().info(f"Fanuc model (RTB model): {self.fanuc_model.name}")

        self.ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Kp = 0.01
        self.target_pose = [245.0792999267578, 239.99853515625, 88.63890075683594, 179.0, 0.0023873525205999613, -0.0001757410354912281]
        self.target_joint = [44.543113708496094, 15.211978912353516, -22.45157814025879, 0.7657142281532288, -68.25440979003906, -44.83275604248047]
        self.home_pose = [290.0001525878906, 99.65259552001953, 88.6458740234375, 179.0, 0.0015575372381135821, 0.0010508003178983927]
        self.triggered = False
        movement_timer_period = 1/100     # 100hz
        self.mut_cb_group = MutuallyExclusiveCallbackGroup()

        # real hardware robot arm
        self.bot = robot(self.get_parameter('robot_ip').value)
        
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_movement', self.trigger_move_cb)
        self.movement_timer = self.create_timer(movement_timer_period, self.timer_callback_3, self.mut_cb_group)

    def trigger_move_cb(self, request, response):
        if (request.data):
            self.triggered = True
        else:
            self.triggered = False
        
        # go to home position
        self.bot.write_cartesian_position(coords=self.home_pose, blocking=False)
        time.sleep(2)
        while (self.bot.is_moving()):
            time.sleep(0.1)

        response.success = True
        response.message = "trigger successfull"
        return response

    def timer_callback_3(self):
        if (self.triggered):
            
            # compute error (position error)
            cur_jp = self.bot.read_current_joint_position()
            cur_error = np.subtract(np.array(self.target_joint), np.array(cur_jp))
            # cur_error[3:] = [0.0, 0.0, 0.0]
            # print(cur_error)
            joint_error = (cur_error * self.Kp)
                
            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()
            # current joint position (deg to rad) + J23 coupling
            rad_arr = np.deg2rad(cur_joint_pose)
            # remove coupling - J[3]' = J[3] + J[2]
            rad_arr[2] = rad_arr[2] + rad_arr[1]

            # ee velocity to joint velocity (for current joint angles)
            current_jacobian = self.fanuc_model.jacobe(q=np.array(rad_arr))             # 6x6 matrix
            joint_vels = (np.linalg.pinv(current_jacobian) @ np.array(self.ee_vel).T)  # 6x6 @ 6x1 => 6x1
            joint_vels = joint_vels.flatten()      # [Vj1, Vj2, Vj3, Vj4, Vj5, Vj6]

            # some filtering has to be done on the joint velocities before adding to the current joint positioni
            ### TODO: filter to joint_vels (or some PID control) - not sure
            
            # worked after inverting the target velocity of joint 2 (may be it is inverted)
            joint_vels[1] *= -1
            # joint_error[1] *= -1

            # add that to current joint position
            target_rad_arr = np.add(rad_arr, joint_error)
            
            # adding coupling - J[3]' = J[3] - J[2]
            target_rad_arr[2] = target_rad_arr[2] - target_rad_arr[1]
            target_joint_pose = np.rad2deg(target_rad_arr).tolist()

            self.get_logger().info(f"target ee vel: {joint_error}")

            # write register and sync-movement
            self.get_logger().info(f"Computed Joint Position: {target_joint_pose}")
            self.bot.write_joint_pose(target_joint_pose, blocking=False)


def main(args=None):
    rclpy.init(args=args)

    ee_twist_node = Move2PoseCMD()

    rclpy.spin(ee_twist_node)

    ee_twist_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    