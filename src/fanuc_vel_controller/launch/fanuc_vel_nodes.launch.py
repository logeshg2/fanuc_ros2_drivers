import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'fanuc_vel_controller'

    # robot_name_launch_arg = DeclareLaunchArgument(
    #     'robot_name',
    #     default_value='noName',
    #     description="Name of the robot these nodes will be attached to"
    # )
    # robot_ip_launch_arg = DeclareLaunchArgument(
    #     'robot_ip',
    #     default_value = '172.29.208.1',
    #     description="IP address of the robot these nodes will be attached to"
    # )
    # robot_name = LaunchConfiguration('robot_name')
    # robot_ip = LaunchConfiguration('robot_ip')

    aruco_node = Node(
        package=package_name,
        executable='aruco_node.py',
        # parameters=[{"robot_ip": robot_ip,
                    #  "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    aruco_tracker = Node(
        package=package_name,
        executable='aruco_tracker.py',
        # parameters=[{"robot_ip": robot_ip,
                    #  "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )

    return launch.LaunchDescription([
        aruco_node,
        aruco_tracker
    ])