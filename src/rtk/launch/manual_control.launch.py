from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port for Arduino (for example /dev/ttyACM0)",
    )
    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="115200",
        description="Baud rate",
    )
    step_arg = DeclareLaunchArgument(
        "step",
        default_value="10",
        description="Delta per keypress",
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout",
        default_value="0.2",
        description="Serial timeout seconds",
    )
    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="joint_steps_abs",
        description="Topic for absolute joint step targets",
    )

    manual_control_node = Node(
        package="rtk",
        executable="manual_control",
        name="manual_control",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--step",
            LaunchConfiguration("step"),
            "--topic",
            LaunchConfiguration("topic"),
        ],
    )

    serial_bridge_node = Node(
        package="rtk",
        executable="serial_step_bridge",
        name="serial_step_bridge",
        output="screen",
        arguments=[
            "--port",
            LaunchConfiguration("port"),
            "--baud",
            LaunchConfiguration("baud"),
            "--timeout",
            LaunchConfiguration("timeout"),
            "--topic",
            LaunchConfiguration("topic"),
        ],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        step_arg,
        timeout_arg,
        topic_arg,
        manual_control_node,
        serial_bridge_node,
    ])
