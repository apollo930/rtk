#!/usr/bin/env python3
"""
Interactive keyboard publisher for 6-DOF absolute step targets.

Direct keyboard control (no Enter needed).

Default key bindings:
    q/a -> J1 +/-
    w/s -> J2 +/-
    e/d -> J3 +/-
    r/f -> J4 +/-
    t/g -> J5 +/-
    y/h -> J6 +/-

Other keys:
    ?     show help
    x     quit
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time
from collections.abc import Iterator
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int32MultiArray

if os.name == "nt":
    import msvcrt  # type: ignore[attr-defined]
else:
    import termios
    import tty


KEYMAP: dict[str, tuple[int, int]] = {
    "q": (1, +1),
    "a": (1, -1),
    "w": (2, +1),
    "s": (2, -1),
    "e": (3, +1),
    "d": (3, -1),
    "r": (4, +1),
    "f": (4, -1),
    "t": (5, +1),
    "g": (5, -1),
    "y": (6, +1),
    "h": (6, -1),
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct keyboard publisher for 6-DOF absolute targets")
    parser.add_argument("--step", type=int, default=10, help="Delta per keypress (default: 10)")
    parser.add_argument(
        "--topic",
        default="joint_steps_abs",
        help="Topic to publish Int32MultiArray absolute targets (default: joint_steps_abs)",
    )
    if argv is None:
        argv = sys.argv
    return parser.parse_args(remove_ros_args(argv)[1:])


@contextmanager
def keyboard_mode(input_fd: int) -> Iterator[None]:
    if os.name == "nt":
        yield
        return

    old_settings = termios.tcgetattr(input_fd)
    try:
        tty.setraw(input_fd)
        yield
    finally:
        termios.tcsetattr(input_fd, termios.TCSADRAIN, old_settings)


def read_key(input_fd: int) -> str:
    if os.name == "nt":
        return msvcrt.getwch()
    return os.read(input_fd, 1).decode("utf-8", errors="ignore")


def read_key_nonblocking(input_fd: int, timeout_s: float = 0.05) -> str | None:
    if os.name == "nt":
        if msvcrt.kbhit():
            return msvcrt.getwch()
        time.sleep(timeout_s)
        return None

    ready, _, _ = select.select([input_fd], [], [], timeout_s)
    if ready:
        return read_key(input_fd)
    return None


def resolve_input_fd() -> int | None:
    if os.name == "nt":
        return None

    if sys.stdin.isatty():
        return sys.stdin.fileno()

    try:
        return os.open("/dev/tty", os.O_RDONLY)
    except OSError:
        return None


def command_from_key(key: str, step: int) -> str | None:
    mapped = KEYMAP.get(key.lower())
    if mapped is None:
        return None

    joint, direction = mapped
    delta = direction * step
    return f"J{joint}:{delta:+d}"


def print_help(step: int) -> None:
    print("Keyboard control active (no Enter needed, publishing absolute targets):")
    print(f"  q/a -> J1 +/-{step}")
    print(f"  w/s -> J2 +/-{step}")
    print(f"  e/d -> J3 +/-{step}")
    print(f"  r/f -> J4 +/-{step}")
    print(f"  t/g -> J5 +/-{step}")
    print(f"  y/h -> J6 +/-{step}")
    print("  ?   -> show this help")
    print("  x   -> quit")


class ManualControlNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("manual_control")
        self.publisher = self.create_publisher(Int32MultiArray, topic, 10)

    def publish_targets(self, targets: list[int]) -> None:
        msg = Int32MultiArray()
        msg.data = targets
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    cli = parse_args(args)
    node = ManualControlNode(cli.topic)
    input_fd: int | None = resolve_input_fd()
    should_close_input_fd = bool(
        os.name != "nt" and input_fd is not None and not sys.stdin.isatty()
    )

    if cli.step <= 0:
        node.get_logger().error("--step must be a positive integer")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if os.name != "nt" and input_fd is None:
        node.get_logger().error(
            "Keyboard mode requires a TTY terminal (stdin is not interactive and /dev/tty is unavailable)."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    targets = [0, 0, 0, 0, 0, 0]
    node.publish_targets(targets)
    node.get_logger().info(f"Publishing to topic: {cli.topic}")
    print_help(cli.step)

    try:
        with keyboard_mode(input_fd if input_fd is not None else 0):
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                try:
                    key = read_key_nonblocking(input_fd if input_fd is not None else 0)
                except (EOFError, KeyboardInterrupt):
                    print("\nExiting.")
                    break

                if not key:
                    continue

                if key in {"\x03", "\x1b", "x", "X"}:  # Ctrl+C, Esc, x
                    print("\nExiting.")
                    break

                if key == "?":
                    print()
                    print_help(cli.step)
                    continue

                cmd = command_from_key(key, cli.step)
                if cmd is None:
                    continue

                joint_text, delta_text = cmd.split(":", maxsplit=1)
                joint_index = int(joint_text[1:]) - 1
                delta = int(delta_text)
                targets[joint_index] += delta

                node.publish_targets(targets)
                print(
                    "\rJ:" + ",".join(str(value) for value in targets),
                    end="",
                    flush=True,
                )
    finally:
        if should_close_input_fd and input_fd is not None:
            os.close(input_fd)

    node.destroy_node()
    rclpy.shutdown()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
