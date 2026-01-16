#!/usr/bin/python3
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

import cv2
import numpy as np

from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt, QObject, Signal


# =======================
# Qt signal helper
# =======================
class ImageSignal(QObject):
    image_received = Signal(QImage)


# =======================
# Qt Viewer
# =======================
class ImageViewer(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 Image Viewer + Rotate")
        self.label = QLabel(alignment=Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        self.signal = ImageSignal()
        self.signal.image_received.connect(self.update_image)

        print("[ImageViewer] GUI initialized")

    def update_image(self, qimg: QImage):
        pixmap = QPixmap.fromImage(qimg)
        self.label.setPixmap(
            pixmap.scaled(
                self.label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )


# =======================
# ROS Node: rotation + image
# =======================
class RotateAndImageNode(Node):
    def __init__(self, viewer: ImageViewer):
        super().__init__('rotate_and_image_node')

        self.viewer = viewer
        self.vehicle_name = os.getenv('VEHICLE_NAME', 'duckiebot')

        self.get_logger().info(f"Node started for {self.vehicle_name}")

        # -------- Wheels publisher --------
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )

        self.left_speed = 0.08
        self.right_speed = -0.08  # spin in place

        self.rotate_timer = self.create_timer(0.1, self.rotate)

        # -------- Image subscriber --------
        qos = QoSProfile(depth=10)
        image_topic = f'/{self.vehicle_name}/image/compressed'

        self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            qos
        )

        self.get_logger().info(f"Subscribed to {image_topic}")

        # -------- ArUco --------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    # -------------------
    # Rotation
    # -------------------
    def rotate(self):
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rotate'
        msg.vel_left = self.left_speed
        msg.vel_right = self.right_speed
        self.wheels_pub.publish(msg)

    # -------------------
    # Image callback
    # -------------------
    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().error("Failed to decode image")
            return

        # --- Detect red ---
        red_detected = self.detect_red_color(frame)

        # --- Detect ArUco ---
        aruco_ids = self.detect_aruco(frame)

        # --- Draw overlay ---
        self.draw_overlay(frame, red_detected, aruco_ids)

        # --- Convert to Qt ---
        h, w, _ = frame.shape
        qimg = QImage(
            frame.data,
            w,
            h,
            3 * w,
            QImage.Format_RGB888
        ).rgbSwapped()

        self.viewer.signal.image_received.emit(qimg)

    # -------------------
    # Red detection
    # -------------------
    def detect_red_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 140, 120])
        upper_red1 = np.array([7, 255, 255])
        lower_red2 = np.array([165, 140, 120])
        upper_red2 = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lower_red1, upper_red1) | \
               cv2.inRange(hsv, lower_red2, upper_red2)

        red_ratio = cv2.countNonZero(mask) / (frame.shape[0] * frame.shape[1])
        return red_ratio > 0.01

    # -------------------
    # ArUco detection
    # -------------------
    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return ids.flatten().tolist()

        return []

    # -------------------
    # Overlay text
    # -------------------
    def draw_overlay(self, frame, red_detected, aruco_ids):
        y = 30
        dy = 30
        font = cv2.FONT_HERSHEY_SIMPLEX

        red_text = f"RED: {'YES' if red_detected else 'NO'}"
        cv2.putText(frame, red_text, (10, y), font, 0.8,
                    (0, 0, 255) if red_detected else (200, 200, 200), 2)

        y += dy

        if aruco_ids:
            aruco_text = f"ARUCO: YES (id={aruco_ids})"
            color = (0, 255, 0)
        else:
            aruco_text = "ARUCO: NO"
            color = (200, 200, 200)

        cv2.putText(frame, aruco_text, (10, y), font, 0.8, color, 2)

    # -------------------
    # Stop on shutdown
    # -------------------
    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.wheels_pub.publish(msg)

    def destroy_node(self):
        self.stop()
        super().destroy_node()


# =======================
# Main
# =======================
def main():
    rclpy.init()

    app = QApplication(sys.argv)
    viewer = ImageViewer()
    viewer.show()

    node = RotateAndImageNode(viewer)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
