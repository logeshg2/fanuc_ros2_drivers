from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    robot_description = ParameterValue(
    Command([
        'xacro ',
        os.path.join(
            get_package_share_directory('fanuc_moveit_config'),
            'config',
            'fanuc_lrmate200id4s.urdf.xacro'
        )
    ]),
    value_type=str
)
    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'fanuc'
            ],
            output='screen'
        ),

        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(
                    get_package_share_directory('fanuc_moveit_config'),
                    'config',
                    'ros2_controllers.yaml'
                )
            ],
            output='screen'
        ),

        # Spawn controllers
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller_manager',
                'spawner', 'joint_state_broadcaster'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller_manager',
                'spawner', 'fanuc_arm_controller'
            ],
            output='screen'
        ),
    ])
