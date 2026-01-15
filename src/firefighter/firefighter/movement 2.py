import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern


class TofNode(Node):
    LED_COLORS = {
        0: ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # off
        1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # red
        2: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # green
        3: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)   # blue
    }

    def __init__(self):
        super().__init__('tof_node')

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'unknown_vehicle')

        # LED publisher
        self.led_publisher = self.create_publisher(
            LEDPattern,
            f'/{self.vehicle_name}/led_pattern',
            1
        )


        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )


        self.tof_sub = self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.check_range,
            10
        )


        self.led_timer = self.create_timer(0.5, self.publish_led_pattern)
        self.led_state = 0
        self.led_counter = 0


        self.startup_rotations_total = 3
        self.startup_rotations_done = 0
        self.rotating = True

        self.rotation_timer = self.create_timer(
            0.1,
            self.startup_rotation_step
        )

        self.get_logger().info(
            f"TofNode started for vehicle: {self.vehicle_name}"
        )
        self.get_logger().info("Startup rotation sequence beginning")


    def change_color(self, color):
        if self.led_state == color:
            return
        self.led_counter = 0
        self.led_state = color
        self.publish_led_pattern()
        self.led_timer.reset()

    def publish_led_pattern(self):
        msg = LEDPattern()
        if self.led_counter % 2:
            pattern = self.LED_COLORS[0]
        else:
            pattern = self.LED_COLORS[self.led_state]

        msg.rgb_vals = [pattern] * 5
        self.led_counter += 1
        self.led_publisher.publish(msg)


    def startup_rotation_step(self):
        if self.startup_rotations_done >= self.startup_rotations_total:
            self.run_wheels("rotation_complete", 0.0, 0.0)
            self.rotating = False
            self.rotation_timer.cancel()
            self.change_color(3)  # blue = ready
            self.get_logger().info("Startup rotations completed")
            return

        self.get_logger().info(
            f"Rotation {self.startup_rotations_done + 1}/"
            f"{self.startup_rotations_total}"
        )

        self.change_color(2)  # green while rotating
        self.run_wheels("rotate", -0.3, 0.3)

        # stop after ~160 degrees (~1s)
        self.create_timer(1.0, self.finish_single_rotation)

    def finish_single_rotation(self):
        self.run_wheels("rotate_stop", 0.0, 0.0)
        self.startup_rotations_done += 1


    def check_range(self, msg: Range):
        if self.rotating:
            return

        distance = msg.range
        self.get_logger().info(
            f"Received range data: {distance:.3f} m"
        )

        if distance >= 0.7:
            self.move_forward()
        else:
            self.stop()


    def move_forward(self):
        self.change_color(2)
        self.run_wheels("forward", 0.5, 0.5)

    def stop(self):
        self.change_color(1)
        self.run_wheels("stop", 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = vel_left
        msg.vel_right = vel_right

        self.wheels_pub.publish(msg)

        self.get_logger().debug(
            f"WheelsCmd | {frame_id} | "
            f"L={vel_left:.2f} R={vel_right:.2f}"
        )


    def on_shutdown(self):
        self.get_logger().info("Shutting down safely")
        self.run_wheels("shutdown", 0.0, 0.0)
        self.change_color(0)


def main():
    rclpy.init()
    node = TofNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
