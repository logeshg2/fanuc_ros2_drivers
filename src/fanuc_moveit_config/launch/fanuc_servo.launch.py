import yaml
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    default_model_path = get_package_share_path('moveit_fanuc_description') / 'urdf/lrmate200id4s.xacro'
    default_rviz_config_path = get_package_share_path('fanuc_moveit_config') / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # Get parameters for the Servo node
    servo_yaml = load_yaml("fanuc_moveit_config", "config/fanuc_real_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    moveit_config = (
        MoveItConfigsBuilder("fanuc")
        .robot_description(file_path="config/fanuc_lrmate200id4s.urdf.xacro")
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('fanuc_moveit_config')/ "launch/move_group.launch.py")
        ),
    )

    # static publisher launcher of hand eye calibration
    hand_eye_static_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('fanuc_statemachine')/ "launch/hand_eye_calibration_rs.launch.py")
        ),
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        move_group,
        hand_eye_static_launcher,
        # joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        servo_node
    ])
