#!/home/logesh/robotic_toolbox_ws/toolbox_env/bin/python3
# NOTE: I am using python3-venv for robotics-toolbox - please set to default py-interpreter if you install rtb in global env. 

import os
import time
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH

# define the  robot class
class Fanuc(DHRobot):
    """
        -----
        Fanuc LR Mate 200iD 4s robot's model using DH parameters
        -----

        Description:
            Robot model developed using Robotics Toolbox by Peter Corke. This is the Python implementation of the robot model.
            For DH parameters refer this site: https://www.fanucamerica.com/cmsmedia/datasheets/LR%20Mate%20200iD%20Series_187.pdf and the below model also.
            
            The script / Fanuc Model also contains some extra abstracted feature for simpler use like move_to(), get_curpos().

        Author:
            - Logesh G
    """

    def __init__(self):
        # DH parameters as revolute joints
        Links = [
            RevoluteDH(
                d=0,
                a=0,
                alpha=np.pi/2,
                qlim=np.array(np.radians([-170, 170]))
            ),
            RevoluteDH(
                d=0.0,
                a=0.260,
                alpha=0,
                offset=np.pi/2,
                qlim=np.array(np.radians([-100, 145]))
            ),
            RevoluteDH(
                d=0.0,
                a=0.020,
                alpha=-np.pi/2,
                qlim=np.array(np.radians([-140, 140]))      # -140, 200
            ),
            RevoluteDH(
                d=-0.290,
                a=0.0,
                alpha=np.pi/2,
                qlim=np.array(np.radians([-150, 150]))      # -180, 180
            ),
            RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=-np.pi/2,
                qlim=np.array(np.radians([-120, 120]))
            ),
            RevoluteDH(
                d=-0.070,
                a=0.0,
                alpha=np.pi,
                qlim=np.array(np.radians([-180, 180]))      # -360, 360
            ),
        ]

        # tool = sm.base.transl(0, 0, 0.13)                   # no rotation for servo gripper  [tool_offset = 13cm] -> tip of the gripper
        # tool = sm.base.transl(0, 0, 0)                      # aruco board tool (for calibration)

        super().__init__(
            links=Links,
            name="LR_Mate_200iD_4s",
            manufacturer="Fanuc",
            # tool = tool
        )

        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        # self.PLC_IP = '192.168.1.7'                         # ASRS PLC IP (Laptop -> ASRS PLC -> Fanuc PLC)
        self.cur_tf = [0, 0, 0, 0, 0, 0]
        self.show_plot = True


if __name__ == "__main__":

    robot = Fanuc()
    print(robot)
    # print(robot.jacobe(robot.qz))
    # print(robot.q)

    vel = np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0])

    while True:
        try:
            # print(robot.q)
            joint_vel = np.linalg.pinv(robot.jacobe(robot.q)) @ vel.T
            joint_vel = joint_vel.flatten()

            # use robot.jacobe - end effector vel jacobian matrix
            # use robot.jacob0 - world coord vel jacobian matrix

            robot.q = np.add(robot.q, joint_vel)
            
            # print(joint_vel)
            # print(robot.q)
            # break
            # time.sleep(2)
            robot.plot(robot.q, block=False)
            plt.pause(2)
            plt.cla()
        except:
            plt.close('all')
            exit(0)

    '''
    # sample position
    tf = sm.SE3(0.35, 0.0, 0.05) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
    print(tf)
    mask = [1, 1, 1, 1, 1, 1]
    # sample ikine
    sol = robot.ikine_LM(tf, q0=robot.qz, mask=mask, joint_limits=True)
    robot.plot(sol.q, block=True)
    # sample fkine
    print(robot.fkine(sol.q))
    '''
