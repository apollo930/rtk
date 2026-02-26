#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int32MultiArray


class StepsTerminalInputNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("steps_terminal_input")
        self.publisher = self.create_publisher(Int32MultiArray, topic, 10)
        self.topic = topic

    def publish_steps(self, steps: list[int]) -> None:
        msg = Int32MultiArray()
        msg.data = steps
        self.publisher.publish(msg)
        self.get_logger().info(f"Published to {self.topic}: {steps}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read 6 joint step values from terminal and publish Int32MultiArray"
    )
    parser.add_argument(
        "--topic",
        default="joint_steps_abs",
        help="Topic for Int32MultiArray absolute steps (default: joint_steps_abs)",
    )

    if argv is None:
        argv = sys.argv
    return parser.parse_args(remove_ros_args(argv)[1:])


def parse_steps(raw: str) -> list[int] | None:
    cleaned = raw.replace(",", " ").strip()
    parts = [part for part in cleaned.split() if part]
    if len(parts) != 6:
        return None

    try:
        return [int(part) for part in parts]
    except ValueError:
        return None


def main(args: list[str] | None = None) -> int:
    cli = parse_args(args)
    rclpy.init(args=args)
    node = StepsTerminalInputNode(cli.topic)

    print("Enter 6 step values as space/comma separated integers.")
    print("Example: 100 200 -50 0 90 1000")
    print("Type 'q' to quit.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            user_input = input("> steps (j1 j2 j3 j4 j5 j6): ").strip()

            if user_input.lower() in {"q", "quit", "exit"}:
                break

            steps = parse_steps(user_input)
            if steps is None:
                print("Invalid input. Please enter exactly 6 integers.")
                continue

            node.publish_steps(steps)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
