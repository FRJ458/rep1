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

        self.setWindowTitle("ROS2 Image Viewer - Red Detector")
        self.label = QLabel(alignment=Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        self.signal = ImageSignal()
        self.signal.image_received.connect(self.update_image)

        print("[ImageViewer] GUI initialized")

        self.flash_on = False
        self.flash_counter = 0

    def update_image(self, qimg: QImage):

        self.flash_counter += 1
        if self.flash_counter % 10 < 5:
            self.setStyleSheet("background-color: blue;")
        else:
            self.setStyleSheet("background-color: black;")

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

        self.get_logger().info('Image viewer node with red detection started')
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
        red_detected = self.detect_red_color(cv_image)

        if red_detected:
            self.get_logger().info('red detected')

        qimg = QImage(
            cv_image.data,
            w,
            h,
            3 * w,
            QImage.Format_RGB888
        ).rgbSwapped()

        self.viewer.signal.image_received.emit(qimg)

        self.get_logger().debug('Image sent to GUI thread')

    def detect_red_color(self, cv_image):


        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 140, 120])
        upper_red1 = np.array([7, 255, 255])
        lower_red2 = np.array([165, 140, 120])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = cv_image.shape[0] * cv_image.shape[1]
        red_ratio = red_pixels / total_pixels

        return red_ratio > 0.01



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