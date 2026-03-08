from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for Arduino/controller",
    )
    serial_baud_arg = DeclareLaunchArgument(
        "serial_baud",
        default_value="115200",
        description="Serial baud rate",
    )
    serial_timeout_arg = DeclareLaunchArgument(
        "serial_timeout",
        default_value="0.2",
        description="Serial timeout seconds",
    )
    mic_device_index_arg = DeclareLaunchArgument(
        "mic_device_index",
        default_value="7",
        description="Microphone device index for stt node",
    )
    camera_id_arg = DeclareLaunchArgument(
        "camera_id",
        default_value="4",
        description="Camera device index for object_detection node",
    )

    object_detection_node = Node(
        package="rtk",
        executable="object_detection",
        name="object_detection",
        output="screen",
        parameters=[
            {
                "camera_id": ParameterValue(LaunchConfiguration("camera_id"), value_type=int),
            }
        ],
    )

    stt_node = Node(
        package="rtk",
        executable="stt",
        name="speech_brain_node",
        output="screen",
        additional_env={
            "RTK_MIC_DEVICE_INDEX": LaunchConfiguration("mic_device_index"),
        },
    )

    inverse_kinematics_node = Node(
        package="rtk",
        executable="inverse_kinematics",
        name="inverse_kinematics",
        output="screen",
    )

    serial_step_bridge_node = Node(
        package="rtk",
        executable="serial_step_bridge",
        name="serial_step_bridge",
        output="screen",
        arguments=[
            "--port",
            LaunchConfiguration("serial_port"),
            "--baud",
            LaunchConfiguration("serial_baud"),
            "--timeout",
            LaunchConfiguration("serial_timeout"),
            "--topic",
            "joint_steps_abs",
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            serial_baud_arg,
            serial_timeout_arg,
            mic_device_index_arg,
            camera_id_arg,
            object_detection_node,
            stt_node,
            inverse_kinematics_node,
            serial_step_bridge_node,
        ]
    )
