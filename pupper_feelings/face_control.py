import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
import os

from subprocess import check_output
import time


class JoyListener(Node):
    def __init__(self):
        super().__init__("joy_listener")
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.prev_buttons = None  # Store the previous state of all buttons
        self.prev_axes = None
        while not os.path.exists("/dev/tty1"):
            self.get_logger().info("Waiting for /dev/tty1")
            time.sleep(1)
        self.get_logger().info("Initialized face control")

    def run_on_button(self, new_buttons, button_index, fn):
        if self.prev_buttons[button_index] == 0 and new_buttons[button_index] == 1:
            fn()

    def run_on_axes_change(self, new_axes, axis_index, axis_value, fn):
        if (
            self.prev_axes[axis_index] != axis_value
            and new_axes[axis_index] == axis_value
        ):
            fn()

    def run_script(self, script_args):
        package_share_directory = get_package_share_directory("pupper_feelings")
        resources_path = os.path.join(package_share_directory, "resources")
        script_path = os.path.join(resources_path, "ascii.sh")
        script_command = f"{script_path} {resources_path}/{script_args}"
        self.get_logger().info(f"Running script with args: {script_args}")
        try:
            subprocess.run(script_command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to execute script: {e}")

    def display_ip_battery_voltage(self):
        ip_addr = check_output(["hostname", "-I"]).decode("utf-8").strip()
        battery_voltage = (
            check_output(["python3", "/home/pi/utils/check_batt_voltage.py"])
            .decode("utf-8")
            .strip()
        )
        with open("/dev/tty1", "w") as tty:
            tty.write(
                "IP address: "
                + str(ip_addr)
                + "\t"
                + str(battery_voltage)
                + "\t press 'o' to refresh"
            )

    def regular_eyes_and_info(self):
        self.run_script("regular_eyes.txt")
        self.display_ip_battery_voltage()

    def joy_callback(self, msg):
        d_pad_x_axis = 6
        d_pad_y_axis = 7
        tri_button_index = 2
        o_button_index = 1
        x_button_index = 0
        l1_index = 4
        r1_index = 5

        if self.prev_buttons is None:
            self.prev_buttons = msg.buttons
            return

        if self.prev_axes is None:
            self.prev_axes = msg.axes
            return

        self.run_on_button(
            msg.buttons,
            r1_index,
            lambda: self.run_script("ask.txt"),
        )

        self.run_on_button(msg.buttons, l1_index, self.regular_eyes_and_info)
        self.run_on_axes_change(
            msg.axes,
            d_pad_x_axis,
            -1.0,
            lambda: self.run_script("fancy_eyes_right.txt"),
        )
        self.run_on_axes_change(
            msg.axes, d_pad_x_axis, 1.0, lambda: self.run_script("fancy_eyes_left.txt")
        )
        self.run_on_axes_change(
            msg.axes, d_pad_y_axis, 1.0, lambda: self.run_script("fancy_eyes_up.txt")
        )
        self.run_on_axes_change(
            msg.axes, d_pad_y_axis, -1.0, lambda: self.run_script("fancy_eyes_down.txt")
        )

        # Update the previous button states
        self.prev_buttons = msg.buttons
        self.prev_axes = msg.axes


def main(args=None):
    rclpy.init(args=args)
    joy_listener = JoyListener()
    rclpy.spin(joy_listener)
    joy_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
