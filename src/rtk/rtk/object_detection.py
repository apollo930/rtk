#!/usr/bin/env python3
"""
ROS 2 object detection node based on ArUco workspace calibration.

Publishes detected object centers as geometry_msgs/PoseArray in workspace frame.
"""

from __future__ import annotations

import csv
import json
import os
import time
from typing import Any, TextIO

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


LOWER_RED1 = np.array([0, 120, 70])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 70])
UPPER_RED2 = np.array([180, 255, 255])
LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])

CANNY_LOW = 50
CANNY_HIGH = 150
KERNEL = np.ones((5, 5), np.uint8)
MIN_CONTOUR_AREA = 800
ARUCO_DICT = cv2.aruco.DICT_4X4_50

WORKSPACE_SIZE = 20.0
WARP_PX_PER_CM = 30
WARP_SIZE = int(WORKSPACE_SIZE * WARP_PX_PER_CM)
MAX_CONTOUR_AREA_RATIO = 0.25

VIDEO_SOURCE = "4"
CALIBRATION_PATH = os.path.expanduser("~/tmr/src/rtk/calibration.json")
CSV_OUTPUT_PATH = ""
SHOW_DEBUG = True
FRAME_ID = "workspace"
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720


def load_calibration(path: str | None) -> tuple[np.ndarray | None, np.ndarray | None]:
    if not path:
        return None, None

    if not os.path.exists(path):
        return None, None

    try:
        if str(path).lower().endswith(".json"):
            with open(path, "r", encoding="utf-8") as file_handle:
                data = json.load(file_handle)

            camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
            dist_coeffs = np.array(data["distortion_coefficients"], dtype=np.float64)

            if camera_matrix.shape != (3, 3):
                raise ValueError(f"Invalid camera_matrix shape: {camera_matrix.shape}")
            if dist_coeffs.ndim == 2 and dist_coeffs.shape[0] == 1:
                dist_coeffs = dist_coeffs.reshape(-1)

            return camera_matrix, dist_coeffs

        data = np.load(path)
        return data["camera_matrix"], data["dist_coeffs"]
    except Exception:
        return None, None


def compute_marker_centers(corners: list[np.ndarray], ids: np.ndarray) -> dict[int, np.ndarray]:
    centers: dict[int, np.ndarray] = {}
    for index, marker_id in enumerate(ids.flatten()):
        if marker_id in [0, 1, 2, 3]:
            pts = corners[index][0]
            centers[int(marker_id)] = np.mean(pts, axis=0)
    return centers


def compute_homography(marker_centers: dict[int, np.ndarray]) -> np.ndarray | None:
    required_ids = [0, 1, 2, 3]
    if not all(marker_id in marker_centers for marker_id in required_ids):
        return None

    img_pts = np.array(
        [
            marker_centers[0],
            marker_centers[1],
            marker_centers[2],
            marker_centers[3],
        ],
        dtype=np.float32,
    )

    world_pts = np.array(
        [
            [0.0, 0.0],
            [WORKSPACE_SIZE, 0.0],
            [WORKSPACE_SIZE, WORKSPACE_SIZE],
            [0.0, WORKSPACE_SIZE],
        ],
        dtype=np.float32,
    )

    homography, _ = cv2.findHomography(img_pts, world_pts)
    return homography


def draw_workspace(frame: np.ndarray, homography: np.ndarray) -> None:
    inv_h = np.linalg.inv(homography)
    world_corners = np.array(
        [
            [0, 0],
            [WORKSPACE_SIZE, 0],
            [WORKSPACE_SIZE, WORKSPACE_SIZE],
            [0, WORKSPACE_SIZE],
        ],
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    img_corners = cv2.perspectiveTransform(world_corners, inv_h)
    for index in range(4):
        p1 = tuple(img_corners[index][0].astype(int))
        p2 = tuple(img_corners[(index + 1) % 4][0].astype(int))
        cv2.line(frame, p1, p2, (0, 255, 0), 3)

    overlay = frame.copy()
    for line_index in range(21):
        vertical = np.array(
            [[line_index, 0], [line_index, WORKSPACE_SIZE]], dtype=np.float32
        ).reshape(-1, 1, 2)
        vertical_img = cv2.perspectiveTransform(vertical, inv_h)
        cv2.line(
            overlay,
            tuple(vertical_img[0][0].astype(int)),
            tuple(vertical_img[1][0].astype(int)),
            (0, 255, 0),
            1,
        )

        horizontal = np.array(
            [[0, line_index], [WORKSPACE_SIZE, line_index]], dtype=np.float32
        ).reshape(-1, 1, 2)
        horizontal_img = cv2.perspectiveTransform(horizontal, inv_h)
        cv2.line(
            overlay,
            tuple(horizontal_img[0][0].astype(int)),
            tuple(horizontal_img[1][0].astype(int)),
            (0, 255, 0),
            1,
        )

    cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)


def classify_shape(contour: np.ndarray) -> str | None:
    area = cv2.contourArea(contour)
    if area < MIN_CONTOUR_AREA:
        return None

    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
    circularity = 4 * np.pi * area / (perimeter * perimeter + 1e-6)

    _, _, width, height = cv2.boundingRect(contour)
    aspect_ratio = width / float(height)

    if 4 <= len(approx) <= 6 and 0.70 < aspect_ratio < 1.30:
        return "cube"

    if circularity > 0.65:
        return "cylinder"

    return None


def warp_to_workspace(frame: np.ndarray, homography: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    scale = np.array(
        [
            [WARP_PX_PER_CM, 0, 0],
            [0, WARP_PX_PER_CM, 0],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )
    h_img_to_warp = scale @ homography
    warped = cv2.warpPerspective(frame, h_img_to_warp, (WARP_SIZE, WARP_SIZE))
    return warped, h_img_to_warp


class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detection")

        self.declare_parameter("camera_id", int(VIDEO_SOURCE))
        video_param = int(self.get_parameter("camera_id").value)
        calib_param = CALIBRATION_PATH
        csv_path = CSV_OUTPUT_PATH
        self.show_debug = SHOW_DEBUG
        if self.show_debug and not os.environ.get("DISPLAY"):
            self.show_debug = False
            self.get_logger().warn(
                "show_debug requested but DISPLAY is not set; disabling OpenCV windows."
            )
        self.frame_id = FRAME_ID

        if isinstance(video_param, int):
            self.cap = cv2.VideoCapture(video_param, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(video_param)
        else:
            self.cap = cv2.VideoCapture(video_param)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open video source: {video_param}")

        self.camera_matrix, self.dist_coeffs = load_calibration(calib_param)
        if self.camera_matrix is None:
            self.get_logger().warn("Calibration unavailable, running without undistortion.")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.homography: np.ndarray | None = None
        self.frame_count = 0
        self.view_mode = "final"
        self.last_no_marker_log_time = 0.0
        self.last_no_detection_log_time = 0.0

        self.label_pub = self.create_publisher(String, "detected_objects", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.csv_writer: Any | None = None
        self.csv_file: TextIO | None = None
        if csv_path:
            self.csv_file = open(csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["timestamp", "frame", "cx", "cy", "X_cm", "Y_cm"])

        self.get_logger().info("Publishing object labels on detected_objects")
        self.get_logger().info(
            f"Object detection config: video={video_param}, calib={calib_param}, debug={self.show_debug}"
        )
        if not os.path.exists(calib_param):
            self.get_logger().warn(f"Calibration file not found at {calib_param}")

    def process_frame(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        self.frame_count += 1

        if self.camera_matrix is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            centers = compute_marker_centers(corners, ids)
            if self.homography is None:
                self.homography = compute_homography(centers)
                if self.homography is not None:
                    self.get_logger().info("Homography computed successfully.")
        elif time.time() - self.last_no_marker_log_time > 2.0:
            self.get_logger().warn("No ArUco markers detected (need markers 0,1,2,3).")
            self.last_no_marker_log_time = time.time()

        annotated = frame.copy()
        if self.homography is not None:
            draw_workspace(annotated, self.homography)

        detections: list[tuple[float, float, str]] = []
        display_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        display_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        display_edges = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        if self.homography is not None:
            warped, h_img_to_warp = warp_to_workspace(frame, self.homography)
            h_warp_to_img = np.linalg.inv(h_img_to_warp)

            warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
            mask_red = cv2.bitwise_or(
                cv2.inRange(warped_hsv, LOWER_RED1, UPPER_RED1),
                cv2.inRange(warped_hsv, LOWER_RED2, UPPER_RED2),
            )
            mask_blue = cv2.inRange(warped_hsv, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.bitwise_or(mask_red, mask_blue)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

            warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
            gray_blur = cv2.GaussianBlur(warped_gray, (5, 5), 0)
            edges_gray = cv2.Canny(gray_blur, CANNY_LOW, CANNY_HIGH)

            masked = cv2.bitwise_and(warped, warped, mask=mask)
            blur_color = cv2.GaussianBlur(masked, (5, 5), 0)
            edges_color = cv2.Canny(
                cv2.cvtColor(blur_color, cv2.COLOR_BGR2GRAY), CANNY_LOW, CANNY_HIGH
            )
            edges_color = cv2.bitwise_and(edges_color, mask)

            combined = cv2.bitwise_or(edges_gray, edges_color)
            combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, KERNEL)
            contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_contour_area = MAX_CONTOUR_AREA_RATIO * (WARP_SIZE * WARP_SIZE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA or area > max_contour_area:
                    continue

                x, y, width, height = cv2.boundingRect(contour)
                if (
                    x <= 1
                    or y <= 1
                    or (x + width) >= (WARP_SIZE - 1)
                    or (y + height) >= (WARP_SIZE - 1)
                ):
                    continue

                shape = classify_shape(contour)
                if shape is None:
                    continue

                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue

                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                if mask_red[cy, cx] > 0:
                    color = "red"
                elif mask_blue[cy, cx] > 0:
                    color = "blue"
                else:
                    color = "unknown"

                x_cm = round(cx / WARP_PX_PER_CM, 2)
                y_cm = round(cy / WARP_PX_PER_CM, 2)
                label = f"{color} {shape}"
                detections.append((x_cm, y_cm, label))

                if self.show_debug:
                    draw_label = f"{label} ({x_cm},{y_cm}) cm"
                    cv2.drawContours(warped, [contour], -1, (0, 255, 255), 2)
                    cv2.circle(warped, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(
                        warped,
                        draw_label,
                        (max(cx - 70, 5), max(cy - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 0, 0),
                        2,
                    )

                    p_warp = np.array([[[cx, cy]]], dtype=np.float32)
                    p_img = cv2.perspectiveTransform(p_warp, h_warp_to_img)[0][0]
                    ix, iy = int(p_img[0]), int(p_img[1])
                    cv2.circle(annotated, (ix, iy), 5, (255, 0, 0), -1)
                    cv2.putText(
                        annotated,
                        draw_label,
                        (max(ix - 70, 5), max(iy - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 255),
                        2,
                    )

            display_mask = mask
            display_blur = gray_blur
            display_edges = combined

        label_entries: list[str] = []

        for x_cm, y_cm, label in detections:
            label_entries.append(f"{label}:{x_cm:.2f},{y_cm:.2f}")

            if self.csv_writer is not None:
                self.csv_writer.writerow([time.time(), self.frame_count, x_cm, y_cm, x_cm, y_cm])

        label_msg = String()
        label_msg.data = " | ".join(label_entries)
        self.label_pub.publish(label_msg)

        if not detections and time.time() - self.last_no_detection_log_time > 2.0:
            self.get_logger().info("No objects detected in current frame.")
            self.last_no_detection_log_time = time.time()

        if self.show_debug:
            if self.view_mode == "blur":
                display = display_blur
            elif self.view_mode == "edge":
                display = display_edges
            elif self.view_mode == "mask":
                display = display_mask
            else:
                display = annotated

            try:
                cv2.imshow("Workspace Detector", display)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as exc:
                self.get_logger().error(f"OpenCV display error: {exc}. Disabling debug display.")
                self.show_debug = False
                return

            if key == ord("z"):
                self.view_mode = "blur"
            elif key == ord("x"):
                self.view_mode = "edge"
            elif key == ord("c"):
                self.view_mode = "mask"
            elif key == ord("v"):
                self.view_mode = "final"
            elif key in (ord("q"), 27):
                rclpy.shutdown()

    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = ObjectDetectionNode()
    except Exception as exc:
        print(f"Failed to start object_detection node: {exc}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()