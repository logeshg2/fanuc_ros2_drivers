#!/usr/bin/env python3

"""Admittance control example script using end-effector FT sensor"""

import cv2
import time
import pickle
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose
from visual_servoing_pkg.msg import ArucoCorner
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ComDependencies.robot_controller import robot
import pinocchio


class AdmittanceControl(Node):
    def __init__(self):
        super().__init__("admittance_control_node")

        # robot arm controllers
        self.bot = robot("192.168.1.9")
        self.triggered = False
        self.dt = 1.0

        # setup pinocchio
        self.robotModel = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")[0]
        self.robotData = pinocchio.createDatas(self.robotModel)[0]
        self.eeFrameId = self.robotModel.getFrameId("tool0")

        # PID control (TODO: tune this)
        self.KPX = 2*(0.0001)
        self.KIX = 1*(0.000001)
        self.KDX = 0*(0.00001)
        self.KPY = 2*(0.0001)
        self.KIY = 1*(0.000001)
        self.KDY = 0*(0.00001)
        self.KPZ = 2*(0.0001)
        self.KIZ = 1*(0.000001)
        self.KDZ = 0*(0.00001)

        # velocity filter
        self.noPoints = 10
        self.maFilterArr = np.array([
            [0.0 for i in range(self.noPoints)],
            [0.0 for i in range(self.noPoints)],
            [0.0 for i in range(self.noPoints)],
            [0.0 for i in range(self.noPoints)],
            [0.0 for i in range(self.noPoints)],
            [0.0 for i in range(self.noPoints)],
        ])
        self.maIdx = 0

        # force control parameter (compute initial force offset)
        Fz, Mx, My = self.bot.read_force_sensor_values()     # [Fz, Mx, My]
        self.FT_ideal = np.array([0.0, 0.0, Fz, Mx, My, 0.0])

        # mass coefficient matrix
        self.mass = np.array([
            [8, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 8, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 8, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.5, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.5, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        ])
        # damping coefficient matrix
        self.damp = np.array([
            [60, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 60, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 60, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 15, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 15, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 15]
        ])
        self.arm_des_twist = np.zeros((6,1))

        # transform from FT sensor to EE
        self.fTe = np.eye(4)
        # TODO: need to identify translation
        self.fTe[0:3, 0:3] = Rotation.from_euler("xyz", [0, 0, 90], degrees=True).as_matrix()

        # adjoint transformation matrix (to transform force in FT sensor frame to EE frame)
        # NOTE: this adjoint wrench transform is different from twist adjoint transform matrix
        self.AD_fTe  = np.zeros((6,6))
        self.AD_fTe[0:3, 0:3] = self.fTe[0:3, 0:3]
        self.AD_fTe[3:6, 3:6] = self.fTe[0:3, 0:3]
        # TODO: need to add translation part to AD_fTe


        # ros2 comm variables
        self.inc_srv_trig = self.create_service(SetBool, '/trigger_servoing', self.trigger_servoing_cb)
        self.main_timer = self.create_timer(1/20, self.main_timer_cb)


    def trigger_servoing_cb(self, request, response):
        if (request.data):
            self.triggered = True
        else:
            self.triggered = False
        
        # go to tracking position
        # self.bot.write_cartesian_position(coords=self.tracking_pose, blocking=False)
        # time.sleep(2)
        # while (self.bot.is_moving()):
        #     time.sleep(0.1)

        response.success = True
        response.message = "trigger successful"
        return response

    def integrateVel(self, qpos, qvel):
        # update joint position by integrating velocity
        for i in range(6):
            qpos[i] += (self.dt * qvel[i])
        
        return qpos

    def maVelFilter(self, jointVels):
        """
        Function to perform filtering on computed velocity.
        using simple `Moving Average` filter approach here. 
        """
        # TODO: use matrix multiplication to do this - instead of brute for approach

        fVels = np.array([0.0 for i in range(6)])
        for idx in range(6):    # iterate over joints
            self.maFilterArr[idx][self.maIdx] = jointVels[idx]
            fVels[idx] = np.sum(self.maFilterArr[idx]) / self.noPoints
        
        self.maIdx += 1
        self.maIdx %= self.noPoints

        return fVels


    def main_timer_cb(self):
        if (self.triggered):
            # read the current cartesion position
            cur_joint_pose = self.bot.read_current_joint_position()
            # current joint position (deg to rad) + J23 coupling
            rad_arr = np.deg2rad(cur_joint_pose)
            # remove coupling - J[3]' = J[3] + J[2]
            rad_arr[2] = rad_arr[2] + rad_arr[1]

            # read force
            Fz, Mx, My = self.bot.read_force_sensor_values()     # [Fz, Mx, My]
            # print(Fz, Mx, My)
            
            curFT_val = np.array([0.0, 0.0, Fz, Mx, My, 0.0])
            wrench_ext_ft = curFT_val - self.FT_ideal                 # subtract offset force from FT measurement
            wrench_ext_ft = np.array(wrench_ext_ft).reshape((6,1))    # in FT sensor frame

            # wrench from FT sensor frame to ee frame 
            wrench_ext_ee = self.AD_fTe.T @ wrench_ext_ft          # 6x6 @ 6x1

            # compute robot jacobian
            jac = pinocchio.computeFrameJacobian(self.robotModel, self.robotData, np.array(rad_arr), self.eeFrameId)

            arm_des_acc = np.linalg.pinv(self.mass) @ ((-self.damp @ self.arm_des_twist) + wrench_ext_ee)
            self.arm_des_twist = arm_des_acc * 0.1
            # TODO: change the integration (dt) to proper measured dt
            eeVel = self.arm_des_twist.flatten()
            # cartVel[3], cartVel[4] = cartVel[4], cartVel[3]
            # cartVel[3] *= -1
            # cartVel[4] *= -1

            print(np.round(eeVel, 2))
            
            # compute joint velocity from 
            joint_vels = (np.linalg.pinv(jac) @ np.array([eeVel]).reshape((6,1)))
            joint_vels = joint_vels.flatten()

            # compute target joint angle from target velocity
            target_rad_arr = self.integrateVel(qpos=rad_arr, qvel=joint_vels)

            # adding coupling - J[3]' = J[3] - J[2]
            target_rad_arr[2] = target_rad_arr[2] - target_rad_arr[1]
            target_joint_pose = np.rad2deg(target_rad_arr).tolist()

            # write register and sync-movement
            # self.get_logger().info(f"Computed Joint Position: {np.round(target_joint_pose, 4)}")
            self.bot.write_joint_pose(target_joint_pose, blocking=False)

def main():
    rclpy.init()

    try:
        node = AdmittanceControl()
        rclpy.spin(node)
    except Exception as e:
        print(f"Shutting down Admittance Control node:\nException: {e}")
        node.destroy_node()

if __name__ == "__main__":
    main()