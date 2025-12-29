#!/usr/bin/env python3
import sys
import rclpy

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver

from ComDependencies.robot_controller import robot
from std_srvs.srv import SetBool
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class pneumatic_gripper(Node):
    def __init__(self):
        super().__init__('pneumatic_gripper_ctrl_nodes')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')]
        )

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.srv = self.create_service(SetBool, f"{self.get_parameter('robot_name').value}/trigger_air_gripper", self.service_callback)

    def service_callback(self, request, response):
        try:
            if request.data == True:
                # close gripper
                self.bot.air_gripper_control(cmd='close')
                self.get_logger().info('Openning pneumatic (air) gripper')
            else:
                # open gripper
                self.bot.air_gripper_control(cmd='open')
                self.get_logger().info('Closing pneumatic (air) gripper')

            if FANUCethernetipDriver.DEBUG:
                self.get_logger().info('Pneumatic air gripper trigger: {}'.format('Close' if request.data else 'Open'))

            response.success = True 
            response.message = 'Service call successful - gripper trigger'
            return response
        except Exception as e:
            self.get_logger().error(f"Error in air gripper trigger srv: {e}")
            response.success = False 
            response.message = f'Service call Failure (gripper trigger): {e}'
            return response

def main(args=None):
    rclpy.init(args=args)

    air_gripper_node = pneumatic_gripper()

    rclpy.spin(air_gripper_node)

    air_gripper_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()