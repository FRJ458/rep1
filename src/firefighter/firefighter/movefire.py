#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern


class TofBlinkerNode(Node):
    def __init__(self):
        super().__init__('tof_blinker_node')

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'unknown_vehicle')
        self.get_logger().info(f"Starting node for vehicle: {self.vehicle_name}")

        # ----- State -----
        self.obstacle_detected = False
        self.blink_counter = 0

        # ----- Subscriber -----
        self.tof_sub = self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.check_range,
            10
        )

        # ----- Publishers -----
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )

        self.led_pub = self.create_publisher(
            LEDPattern,
            f'/{self.vehicle_name}/led_pattern',
            1
        )

        # ----- Timer for blinking -----
        self.timer = self.create_timer(0.5, self.update_leds)

    # ------------------------------------------------
    # Range callback
    # ------------------------------------------------
    def check_range(self, msg: Range):
        distance = msg.range
        self.get_logger().info(f"Range: {distance:.2f} m")

        if distance >= 0.9:
            self.obstacle_detected = False
            self.move_forward()
        else:
            self.obstacle_detected = True
            self.stop()

    # ------------------------------------------------
    # Wheel commands
    # ------------------------------------------------
    def move_forward(self):
        self.publish_wheels("forward", 0.5, 0.5)

    def stop(self):
        self.publish_wheels("stop", 0.0, 0.0)

    def publish_wheels(self, frame_id, vl, vr):
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = vl
        msg.vel_right = vr
        self.wheels_pub.publish(msg)

    # ------------------------------------------------
    # LED logic
    # ------------------------------------------------
    def update_leds(self):
        msg = LEDPattern()

        if self.obstacle_detected:
            # Blink red
            if self.blink_counter % 2 == 0:
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            self.blink_counter += 1
        else:
            # Safe → steady yellow
            color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        msg.rgb_vals = [color] * 5
        self.led_pub.publish(msg)

    # ------------------------------------------------
    # Shutdown
    # ------------------------------------------------
    def on_shutdown(self):
        self.get_logger().info("Shutting down safely")
        self.stop()


def main():
    rclpy.init()
    node = TofBlinkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
