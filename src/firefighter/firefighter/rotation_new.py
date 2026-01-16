#!/usr/bin/python3
import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

import cv2
import numpy as np

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QLabel,
    QVBoxLayout, QHBoxLayout, QGroupBox
)
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt, QObject, Signal


# =========================
# QT SIGNAL BRIDGE
# =========================
class GuiBridge(QObject):
    image_signal = Signal(int, QImage, bool, str)


# =========================
# GUI BLOCK
# =========================
class ResultBlock(QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        self.image_label = QLabel(alignment=Qt.AlignCenter)
        self.text_label = QLabel("Waiting image...")
        self.text_label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.addWidget(self.image_label)
        layout.addWidget(self.text_label)

    def update(self, qimg, red_found, info):
        pixmap = QPixmap.fromImage(qimg)
        self.image_label.setPixmap(pixmap.scaled(320, 240, Qt.KeepAspectRatio))
        color = "green" if red_found else "red"
        self.text_label.setText(f"<b style='color:{color}'>RED: {red_found}</b><br>{info}")


# =========================
# MAIN GUI
# =========================
class ImageViewer(QWidget):
    def __init__(self, bridge):
        super().__init__()
        self.setWindowTitle("Duckiebot Red + ArUco Search FSM")
        self.blocks = [ResultBlock(f"Scan {i + 1}") for i in range(3)]
        layout = QHBoxLayout(self)
        for block in self.blocks:
            layout.addWidget(block)
        bridge.image_signal.connect(self.update_block)

    def update_block(self, step, qimg, red_found, info):
        if 0 <= step < len(self.blocks):
            self.blocks[step].update(qimg, red_found, info)


# =========================
# ROS NODE
# =========================
class SearchNode(Node):
    def __init__(self, bridge):
        super().__init__('red_aruco_search_node')
        self.bridge = bridge
        self.vehicle = os.getenv('VEHICLE_NAME', 'duckiebot')

        qos = QoSProfile(depth=1)
        self.create_subscription(
            CompressedImage,
            f'/{self.vehicle}/image/compressed',
            self.image_callback,
            qos
        )

        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle}/wheels_cmd',
            10
        )

        # Медленная вращение
        self.left_speed = 0.15
        self.right_speed = -0.15
        self.timer = self.create_timer(0.1, self.rotate)

        # состояние поиска
        self.max_scans = 3
        self.detected_ids = []  # список найденных ArUco
        self.finished = False

        # ArUco setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Папка для сохранения
        self.output_dir = "/workspace/aruco_captures/"
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("Red + ArUco search FSM started and rotating")

    # -------------------------
    def rotate(self):
        if self.finished:
            self.send_wheels(0.0, 0.0)
            return

        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = self.left_speed
        msg.vel_right = self.right_speed
        self.wheels_pub.publish(msg)

    # -------------------------
    def image_callback(self, msg):
        cv_img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_img is None:
            return

        # --- RED DETECTION ---
        red_found, ratio = self.detect_red(cv_img)

        # --- ARUCO DETECTION ---
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        aruco_info = ""
        new_id = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
            # ищем первую метку, которой ещё нет в detected_ids
            for i in ids.flatten():
                if i not in self.detected_ids:
                    self.detected_ids.append(i)
                    new_id = i
                    break
            aruco_info = f"ArUco IDs: {ids.flatten().tolist()}"

        # --- GUI IMAGE ---
        h, w, _ = cv_img.shape
        qimg = QImage(cv_img.data, w, h, 3 * w, QImage.Format_RGB888).rgbSwapped().copy()

        # Определяем в какой блок поставить изображение
        if new_id is not None:
            step = len(self.detected_ids) - 1
            info = f"ratio={ratio:.4f} {aruco_info}"
            self.bridge.image_signal.emit(step, qimg, red_found, info)

            # --- SAVE IMAGE ---
            filename = os.path.join(self.output_dir, f"scan_{step + 1}_id{new_id}.png")
            cv2.putText(cv_img, f"ID: {new_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2)
            cv2.imwrite(filename, cv_img)
            self.get_logger().info(f"Saved image {filename}")

        # Останавливаем после трёх уникальных ArUco
        if len(self.detected_ids) == self.max_scans:
            self.get_logger().info("3 unique ArUco markers captured, stopping rotation")
            self.finished = True

    # -------------------------
    def send_wheels(self, left, right):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = left
        msg.vel_right = right
        self.wheels_pub.publish(msg)

    # -------------------------
    def detect_red(self, cv_img):
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 | mask2
        ratio = np.sum(mask > 0) / (cv_img.shape[0] * cv_img.shape[1])
        return ratio > 0.01, ratio  # True если >1% кадра красного


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    bridge = GuiBridge()
    viewer = ImageViewer(bridge)
    viewer.show()
    node = SearchNode(bridge)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
