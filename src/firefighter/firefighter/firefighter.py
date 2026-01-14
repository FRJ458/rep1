#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.counter = 0
        self.timer = self.create_timer(0.5, self.publish_pattern)  # мигание каждые 0.5 сек

    def publish_pattern(self):
        msg = LEDPattern()

        if self.counter % 2 == 0:
            # включен красный
            pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            # выключен
            pattern = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        msg.rgb_vals = [pattern] * 5
        self.publisher.publish(msg)
        self.counter += 1


def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
