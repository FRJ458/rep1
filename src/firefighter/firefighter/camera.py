#!/usr/bin/python3

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile

import cv2
import numpy as np

from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt, QObject, Signal


class ImageSignal(QObject):
    image_received = Signal(QImage)


class ImageViewer(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 Image Viewer")
        self.label = QLabel(alignment=Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        self.signal = ImageSignal()
        self.signal.image_received.connect(self.update_image)

        print("[ImageViewer] GUI initialized")

    def update_image(self, qimg: QImage):
        # GUI-лог лучше минимальный
        pixmap = QPixmap.fromImage(qimg)
        self.label.setPixmap(
            pixmap.scaled(
                self.label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )


class ImageNode(Node):
    def __init__(self, viewer: ImageViewer):
        super().__init__('image_viewer_node')
        self.viewer = viewer

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'duckiebot')
        qos = QoSProfile(depth=10)

        topic = f'/{self.vehicle_name}/image/compressed'

        self.get_logger().info('Image viewer node started')
        self.get_logger().info(f'Subscribing to topic: {topic}')

        self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            qos
        )

        self.frame_counter = 0

    def image_callback(self, msg: CompressedImage):
        self.frame_counter += 1

        self.get_logger().debug(
            f'Received image #{self.frame_counter}, '
            f'size={len(msg.data)} bytes'
        )

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            self.get_logger().error('Failed to decode image')
            return

        h, w, _ = cv_image.shape
        self.get_logger().debug(f'Decoded image resolution: {w}x{h}')

        qimg = QImage(
            cv_image.data,
            w,
            h,
            3 * w,
            QImage.Format_RGB888
        ).rgbSwapped()

        self.viewer.signal.image_received.emit(qimg)

        self.get_logger().debug('Image sent to GUI thread')


def main():
    rclpy.init()

    print("[Main] Starting Qt application")
    app = QApplication(sys.argv)

    viewer = ImageViewer()
    viewer.show()

    node = ImageNode(viewer)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    print("[Main] ROS spinning in background thread")

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
