#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class TofNode(Node):
    LED_COLORS = {
        0: ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
        1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
        2: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
        3: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
    }

    def __init__(self):
        super().__init__('tof_node')
        self.vehicle_name = os.getenv('VEHICLE_NAME', 'unknown_vehicle')
        self.led_publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.get_logger().info(f"Initializing TofNode for vehicle: {self.vehicle_name}")

        # Subscriber to the range sensor
        self.tof_sub = self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.check_range,
            10
        )
        self.get_logger().info(f"Subscribed to /{self.vehicle_name}/range topic")

        # Publisher for wheel commands
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )
        self.get_logger().info(f"Publisher initialized for /{self.vehicle_name}/wheels_cmd topic")
        self.led_timer = self.create_timer(0.5, self.publish_led_pattern)
        self.led_state = 0  # do not change directly, only with self.change_color()
        self.led_counter = 0

    def change_color(self, color):
        if self.led_state == color:
            return
        self.led_counter = 0
        self.led_state = color
        self.publish_led_pattern()
        self.led_timer.reset()

    def check_range(self, msg: Range):
        distance = msg.range
        self.get_logger().info(f"Received range data: {distance:.3f} meters")

        if distance >= 0.7:
            self.get_logger().info("Distance is safe. Moving forward.")
            self.move_forward()
        else:
            self.get_logger().warn("Distance too short! Stopping the vehicle.")
            self.stop()

    def publish_led_pattern(self):
        msg = LEDPattern()
        if self.led_counter % 2:
            pattern = self.LED_COLORS[0]
        else:
            pattern = self.LED_COLORS[self.led_state]
        msg.rgb_vals = [pattern] * 5
        self.led_counter += 1
        self.led_publisher.publish(msg)

    def move_forward(self):
        self.get_logger().debug("Executing move_forward command")
        self.change_color(2)
        self.run_wheels('forward_command', 0.5, 0.5)

    def stop(self):
        self.change_color(1)
        self.get_logger().debug("Executing stop command")
        self.run_wheels('stop_command', 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)
        self.get_logger().info(
            f"Published WheelsCmdStamped -> frame_id: {frame_id}, "
            f"vel_left: {vel_left:.2f}, vel_right: {vel_right:.2f}"
        )

    def on_shutdown(self):
        self.get_logger().info("Node shutting down: stopping wheels for safety")
        self.stop()
        self.change_color(0)


def main():
    rclpy.init()
    tof_node = TofNode()
    try:
        tof_node.get_logger().info("TofNode is now spinning. Waiting for sensor data...")
        rclpy.spin(tof_node)
    except KeyboardInterrupt:
        tof_node.get_logger().info("KeyboardInterrupt received. Stopping node...")
    finally:
        tof_node.on_shutdown()
        tof_node.destroy_node()
        rclpy.shutdown()
        print("ROS2 shutdown complete.")


if __name__ == '__main__':
    main()
