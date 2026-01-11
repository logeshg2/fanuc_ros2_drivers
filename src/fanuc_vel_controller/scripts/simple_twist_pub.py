import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class publisher_node(Node):
    def __init__(self):
        super().__init__("pub_node")

        self.publisher = self.create_publisher(TwistStamped, '/fanuc_servo/delta_twist_cmds', 10)
    
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        self.create_timer(1/20, self.callback_)

    def callback_(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = 0.01
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

if __name__ == "__main__":
    rclpy.init()

    node = publisher_node()
    rclpy.spin(node)
