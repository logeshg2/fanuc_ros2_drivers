#!/usr/bin/env python3
import sys
import os
import rclpy

import ComDependencies.FANUCethernetipDriver as FANUCethernetipDriver

from ComDependencies.robot_controller import robot
from fanuc_interfaces.action import JointPose
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class follow_joint_server(Node):
    def __init__(self):
        super().__init__('follow_joint_server')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.goal = FollowJointTrajectory.Goal()
        self.bot = robot(self.get_parameter('robot_ip').value)

        self._action_server = ActionServer(self, FollowJointTrajectory, f"/fanuc_arm_controller/follow_joint_trajectory", 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        # cancel_callback = self.cancel_callback
                                        )

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 

        self.get_logger().info(f"Goal received: {self.goal}")

        
        # FIX!! This is ugly.. Put into a list.any()? Switch is also faster
        # Check that it recieved a valid goal
        # if self.goal.joint1 > 179.9 or self.goal.joint1 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        
        # elif self.goal.joint2 > 179.9 or self.goal.joint2 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        
        # elif self.goal.joint3 > 179.9 or self.goal.joint3 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        
        # elif self.goal.joint4 > 179.9 or self.goal.joint4 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        
        # elif self.goal.joint5 > 179.9 or self.goal.joint5 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        
        # elif self.goal.joint6 > 179.9 or self.goal.joint6 < -179.9:
        #     self.get_logger().info('Invalid request')
        #     return GoalResponse.REJECT
        # else:
        #     self.get_logger().info('Joint goal recieved: '+ str(self.goal))
        #     return GoalResponse.ACCEPT
        return GoalResponse.ACCEPT
                
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        if self.goal == None:
            self.get_logger().info('No goal to cancel...')
            return CancelResponse.REJECT
        else:
            self.get_logger().info('Received cancel request')
            goal_handle.canceled()
            return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # try:
        #     feedback_msg = JointPose.Feedback()
        #     feedback_msg.distance_left = self.bot.read_current_joint_position() # starting pose

        #     list = [self.goal.joint1,
        #             self.goal.joint2,
        #             self.goal.joint3,
        #             self.goal.joint4, 
        #             self.goal.joint5,
        #             self.goal.joint6]
            
        #     self.bot.write_joint_pose(list, blocking=False)

        #     while self.bot.is_moving():
        #         # Calculate distance left
        #         feedback_msg.distance_left[0] -= self.goal.joint1
        #         feedback_msg.distance_left[1] -= self.goal.joint2
        #         feedback_msg.distance_left[2] -= self.goal.joint3
        #         feedback_msg.distance_left[3] -= self.goal.joint4
        #         feedback_msg.distance_left[4] -= self.goal.joint5
        #         feedback_msg.distance_left[5] -= self.goal.joint6
        #         goal_handle.publish_feedback(feedback_msg) # Send value

        #         feedback_msg.distance_left = self.bot.read_current_joint_position() # Update cur pos

        #     goal_handle.succeed()
        #     result = JointPose.Result()
        #     result.success = True
        # except:
        #     goal_handle.canceled()
        #     result = JointPose.Result()
        #     result.success = False
        # self.goal = JointPose.Goal() # Reset

        result = FollowJointTrajectory.Result()
        # result. = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    follow_joint_action_server = follow_joint_server()

    rclpy.spin(follow_joint_action_server)

    follow_joint_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
