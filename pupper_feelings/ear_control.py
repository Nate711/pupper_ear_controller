import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pigpio


def lin_map(val, in_min, in_max, out_min, out_max):
    normalized = (val - in_min) / (in_max - in_min)
    return normalized * (out_max - out_min) + out_min


class DualShockServoController(Node):
    def __init__(self):
        super().__init__("dualshock_servo_controller")
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpiod daemon")
            self.destroy_node()

        # GPIO pins for the servos
        self.servo_l = 15
        self.servo_r = 14

    def joy_callback(self, msg):
        l2_value = msg.axes[2]  # from -1 to 1
        r2_value = msg.axes[5]  # from -1 to 1
        l_pulse = int(lin_map(val=l2_value, in_min=-1, in_max=1, out_min=1000, out_max=2000))
        r_pulse = int(lin_map(val=r2_value, in_min=-1, in_max=1, out_min=1000, out_max=2000))

        # Set servo pulse widths
        self.pi.set_servo_pulsewidth(self.servo_l, l_pulse)
        self.pi.set_servo_pulsewidth(self.servo_r, r_pulse)

        self.get_logger().info(f"Set servos to L: {l_pulse}, R: {r_pulse}")


def main(args=None):
    rclpy.init(args=args)
    dualshock_servo_controller = DualShockServoController()
    rclpy.spin(dualshock_servo_controller)
    dualshock_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
