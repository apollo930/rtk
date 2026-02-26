from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    d1_arg = DeclareLaunchArgument("d1", default_value="0.04", description="DH parameter d1")
    a2_arg = DeclareLaunchArgument("a2", default_value="-0.115", description="DH parameter a2")
    a3_arg = DeclareLaunchArgument("a3", default_value="-0.134", description="DH parameter a3")
    d4_arg = DeclareLaunchArgument("d4", default_value="0.0", description="DH parameter d4")
    d6_arg = DeclareLaunchArgument("d6", default_value="0.08", description="DH parameter d6")

    ik_node = Node(
        package="rtk",
        executable="inverse_kinematics",
        name="inverse_kinematics",
        output="screen",
        parameters=[
            {
                "d1": LaunchConfiguration("d1"),
                "a2": LaunchConfiguration("a2"),
                "a3": LaunchConfiguration("a3"),
                "d4": LaunchConfiguration("d4"),
                "d6": LaunchConfiguration("d6"),
            }
        ],
    )

    return LaunchDescription([
        d1_arg,
        a2_arg,
        a3_arg,
        d4_arg,
        d6_arg,
        ik_node,
    ])
