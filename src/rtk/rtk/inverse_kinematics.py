#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray
import math
import time


class PacketMapper(Node):

    def __init__(self):
        super().__init__('packet_mapper')

        # Subscriber
        self.subscription = self.create_subscription(
            PoseArray,
            'pick_coordinates',
            self.callback,
            10
        )

        self.steps_publisher = self.create_publisher(
            Int32MultiArray,
            'joint_steps_abs',
            10
        )

        self.get_logger().info("Packet Mapper Node Started")

        # Grid centers (4x4 base)
        base_grid_x = [4.56125, 8.14375, 11.72625, 15.30875]
        base_grid_y = [5.17375, 8.58125, 11.98875, 15.39625]

        # Packet → Robot command map (4x4 base)
        base_packet_commands = {
            1:  "J=-110,840,350,-80,0,800",
            2:  "J=-110,1040,100,-80,0,800",
            3:  "J=-90,1130,-40,-80,0,800",
            4:  "J=-80,1270,-270,0,0,800",
            5:  "J=20,890,300,0,0,800",
            6:  "J=10,1050,100,0,0,800",
            7:  "J=10,1170,-90,0,0,800",
            8:  "J=10,1320,-340,0,0,800",
            9:  "J=140,750,390,-130,140,800",
            10: "J=110,1040,70,0,160,800",
            11: "J=90,1200,-150,0,160,800",
            12: "J=90,1270,-290,0,160,800",
            13: "J=270,970,220,0,360,800",
            14: "J=230,1060,30,0,360,800",
            15: "J=210,1200,-160,0,360,800",
            16: "J=170,1370,-440,0,360,800",
        }

        # Densify to 7x7 by inserting one midpoint between each base coordinate.
        self.grid_x = self._densify_axis(base_grid_x)
        self.grid_y = self._densify_axis(base_grid_y)
        self.packet_commands = self._densify_packet_commands(base_packet_commands, base_size=4)

    def _densify_axis(self, base_axis):
        dense_axis = []
        for index in range(len(base_axis) - 1):
            start = base_axis[index]
            end = base_axis[index + 1]
            dense_axis.append(start)
            dense_axis.append((start + end) / 2.0)
        dense_axis.append(base_axis[-1])
        return dense_axis

    def _parse_command(self, command_string):
        _, payload = command_string.split("=", 1)
        return [int(value.strip()) for value in payload.split(",")]

    def _format_command(self, step_values):
        return "J=" + ",".join(str(value) for value in step_values)

    def _lerp(self, start, end, ratio):
        return start + (end - start) * ratio

    def _interpolate_steps(self, command_grid, dense_row, dense_col):
        src_row = dense_row / 2.0
        src_col = dense_col / 2.0

        row0 = int(math.floor(src_row))
        col0 = int(math.floor(src_col))
        row1 = min(row0 + 1, len(command_grid) - 1)
        col1 = min(col0 + 1, len(command_grid[0]) - 1)

        row_ratio = src_row - row0
        col_ratio = src_col - col0

        top_left = command_grid[row0][col0]
        top_right = command_grid[row0][col1]
        bottom_left = command_grid[row1][col0]
        bottom_right = command_grid[row1][col1]

        interpolated = []
        for i in range(len(top_left)):
            top = self._lerp(top_left[i], top_right[i], col_ratio)
            bottom = self._lerp(bottom_left[i], bottom_right[i], col_ratio)
            value = self._lerp(top, bottom, row_ratio)
            interpolated.append(int(round(value)))

        return interpolated

    def _densify_packet_commands(self, base_packet_commands, base_size):
        dense_size = base_size * 2 - 1

        command_grid = []
        for row in range(base_size):
            row_commands = []
            for col in range(base_size):
                packet_number = row * base_size + col + 1
                row_commands.append(self._parse_command(base_packet_commands[packet_number]))
            command_grid.append(row_commands)

        dense_commands = {}
        for row in range(dense_size):
            for col in range(dense_size):
                packet_number = row * dense_size + col + 1
                step_values = self._interpolate_steps(command_grid, row, col)
                dense_commands[packet_number] = self._format_command(step_values)

        return dense_commands

    def find_nearest_index(self, value, grid_list):
        """Return index of closest grid coordinate."""
        distances = [abs(value - g) for g in grid_list]
        return distances.index(min(distances))

    def callback(self, msg: PoseArray):

        if not msg.poses:
            return

        # Process each detected object
        for pose in msg.poses:

            x = pose.position.x
            y = pose.position.y

            # Find nearest column and row
            col = self.find_nearest_index(x, self.grid_x)
            row = self.find_nearest_index(y, self.grid_y)

            # Packet numbering (row-major order)
            packet_number = row * len(self.grid_x) + col + 1

            self.get_logger().info(
                f"Object at ({x:.2f}, {y:.2f}) → Packet {packet_number}"
            )

            # Get corresponding robot command
            command_string = self.packet_commands.get(packet_number)

            if command_string:
                try:
                    _, payload = command_string.split("=", 1)
                    step_values = [int(value.strip()) for value in payload.split(",")]
                except (ValueError, IndexError):
                    self.get_logger().warn(f"Invalid command format: {command_string}")
                    continue

                steps_msg = Int32MultiArray()
                steps_msg.data = step_values
                self.steps_publisher.publish(steps_msg)

                self.get_logger().info(
                    f"Published command: {command_string}"
                )
                
                close_grip_val = 130
                time.sleep(3)
                step_values[-1] = close_grip_val  # Close gripper
                steps_msg.data = step_values
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published gripper close command: {command_string}"
                )

                ## return to mean pose
                time.sleep(2)
                steps_msg.data = [0, 0, 0, 0, 0, close_grip_val]
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published return to mean pose command"
                )

                time.sleep(2)
                steps_msg.data = [1030, 0, 0, 0, 0, close_grip_val]
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published return to mean pose command"
                )

                time.sleep(1)
                drop_pose_steps = [1030, 670, 440, -110, 0, close_grip_val]
                steps_msg.data = drop_pose_steps
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published drop pose command"
                )
                time.sleep(2)
                steps_msg.data = [1030, 670, 440, -110, 0, 1000]
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published gripper open command"
                )
                time.sleep(1)
                steps_msg.data = [1030, 0, 0, 0, 0, 1000]
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published return to mean pose command"
                )
                time.sleep(1)
                steps_msg.data = [0, 0, 0, 0, 0, 50]
                self.steps_publisher.publish(steps_msg)
                self.get_logger().info(
                    f"Published return to mean pose command"
                )


def main(args=None):
    rclpy.init(args=args)
    node = PacketMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()