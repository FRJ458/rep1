#!/usr/bin/python3
import os
import sys
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped

import cv2
import numpy as np

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QLabel,
    QVBoxLayout, QHBoxLayout, QGroupBox
)
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt, QObject, Signal


class State(Enum):
    WAIT_IMAGE = 0
    ROTATING = 1
    PAUSE = 2
    DONE = 3

class ResultBlock(QGroupBox):
    def __init__(self, title):
        super().__init__(title)

        self.image_label = QLabel(alignment=Qt.AlignCenter)
        self.text_label = QLabel("Waiting...")
        self.text_label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.image_label)
        layout.addWidget(self.text_label)

    def update(self, qimg, red_found, info):
        pixmap = QPixmap.fromImage(qimg)
        self.image_label.setPixmap(
            pixmap.scaled(320, 240, Qt.KeepAspectRatio)
        )

        color = "green" if red_found else "red"
        self.text_label.setText(
            f"<b style='color:{color}'>RED: {red_found}</b><br>{info}"
        )

class ImageViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Duckiebot Red Search FSM")

        self.blocks = [
            ResultBlock("Scan 1"),
            ResultBlock("Scan 2"),
            ResultBlock("Scan 3"),
        ]

        layout = QHBoxLayout(self)
        for block in self.blocks:
            layout.addWidget(block)


class SearchNode(Node):
    def __init__(self, viewer):
        super().__init__('red_search_node')

        self.viewer = viewer
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

        self.state = State.WAIT_IMAGE
        self.step = 0

        self.get_logger().info("Red search FSM started")


    def image_callback(self, msg):
        if self.state != State.WAIT_IMAGE or self.step >= 3:
            return

        cv_img = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        if cv_img is None:
            self.get_logger().error("Image decode failed")
            return

        red_found, ratio = self.detect_red(cv_img)

        h, w, _ = cv_img.shape
        qimg = QImage(
            cv_img.data,
            w, h,
            3 * w,
            QImage.Format_RGB888
        ).rgbSwapped()

        info = f"step={self.step}, ratio={ratio:.4f}"
        self.viewer.blocks[self.step].update(qimg, red_found, info)

        self.get_logger().info(
            f"[STEP {self.step}] red={red_found}, ratio={ratio:.4f}"
        )

        self.start_rotation()


    def start_rotation(self):
        self.state = State.ROTATING

        self.send_wheels(0.3, -0.3)
        self.get_logger().info("ROTATING for 1.07 sec")

        self.create_timer(1.07, self.stop_and_pause)

    def stop_and_pause(self):
        self.send_wheels(0.0, 0.0)
        self.state = State.PAUSE
        self.get_logger().info("STOP → pause 0.5 sec")

        self.create_timer(0.5, self.next_step)

    def next_step(self):
        self.step += 1
        if self.step >= 3:
            self.state = State.DONE
            self.get_logger().info("FSM finished")
        else:
            self.state = State.WAIT_IMAGE
            self.get_logger().info(f"WAIT_IMAGE step={self.step}")

    def send_wheels(self, left, right):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = left
        msg.vel_right = right
        self.wheels_pub.publish(msg)

    def detect_red(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = (
            cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) |
            cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
        )

        ratio = cv2.countNonZero(mask) / (img.shape[0] * img.shape[1])
        return ratio > 0.01, ratio

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    viewer = ImageViewer()
    viewer.show()

    node = SearchNode(viewer)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()

