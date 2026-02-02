from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    default_model_path = get_package_share_path('moveit_fanuc_description') / 'urdf/lrmate200id4s.urdf'
    default_rviz_config_path = get_package_share_path('fanuc_moveit_config') / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

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

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        move_group,
        # hand_eye_static_launcher,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
