from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    pkg_path = os.path.join(os.path.expanduser('~'), 'tmr/src/rtk')

    urdf_file = os.path.join(pkg_path, 'urdf', 'arm.urdf')

    robot_description = ParameterValue(
    open(urdf_file).read(),
    value_type=str
)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'launch', 'display.rviz')]
        )
    ])