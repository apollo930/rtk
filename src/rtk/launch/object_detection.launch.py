from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    detector_node = Node(
        package="rtk",
        executable="object_detection",
        name="object_detection",
        output="screen",
    )

    return LaunchDescription([detector_node])
