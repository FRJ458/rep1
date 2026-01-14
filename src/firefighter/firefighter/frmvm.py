import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped


class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')
        self.vehicle_name = os.getenv('VEHICLE_NAME', 'unknown_vehicle')

        self.is_moving = True
        self.blink_counter = 0


        self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.range_callback,
            10
        )

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

        self.create_timer(0.5, self.blink_callback)

    def range_callback(self, msg: Range):
        distance = msg.range

        if distance >= 0.9:
            if not self.is_moving:
                self.is_moving = True
            self.move_forward()
        else:
            if self.is_moving:
                self.is_moving = False
                self.blink_counter = 0
            self.stop()

    def blink_callback(self):

        if not self.is_moving:
            if self.blink_counter % 2 == 0:
                pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                pattern = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

            msg = LEDPattern()
            msg.rgb_vals = [pattern] * 5
            self.led_pub.publish(msg)
            self.blink_counter += 1
        else:

            pattern = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            msg = LEDPattern()
            msg.rgb_vals = [pattern] * 5
            self.led_pub.publish(msg)

    def move_forward(self):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'moving'
        wheel_msg.header = header
        wheel_msg.vel_left = 0.5
        wheel_msg.vel_right = 0.5
        self.wheels_pub.publish(wheel_msg)

    def stop(self):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'stopped'
        wheel_msg.header = header
        wheel_msg.vel_left = 0.0
        wheel_msg.vel_right = 0.0
        self.wheels_pub.publish(wheel_msg)


def main():
    rclpy.init()
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()