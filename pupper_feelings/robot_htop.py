"""Notes
I tried curses but it was not any more performant
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateEffortVisualizer(Node):
    def __init__(self):
        super().__init__('robot_htop')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.joint_efforts = {}
        self.count = 0

        self.joint_names =   ["leg_front_r_1",
                            "leg_front_r_2",
                            "leg_front_r_3",
                            "leg_front_l_1",
                            "leg_front_l_2",
                            "leg_front_l_3",
                            "leg_back_r_1",
                            "leg_back_r_2",
                            "leg_back_r_3",
                            "leg_back_l_1",
                            "leg_back_l_2",
                            "leg_back_l_3"]

        self.msg_skip = 20


    def joint_states_callback(self, msg):
        # Ensure there are efforts data
        assert len(msg.effort) == 12
        
        if self.count % self.msg_skip == 0:
            self.joint_efforts = {name: effort for name, effort in zip(msg.name, msg.effort)}
            self.display_efforts()
        self.count += 1

    def display_efforts(self):
        max_effort = 2.0
        
        # Clear the console
        print("\033c", end="")
        
        # Sort joint names alphabetically and display efforts
        for joint_name in self.joint_names:
            effort = self.joint_efforts[joint_name]
            bar_length = int((effort / max_effort) * 50)  # Normalize and scale to 50 characters
            bar = '=' * bar_length if effort >= 0 else '-' * -bar_length
            print(f"{joint_name:15}: [{bar:<50}] {effort:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateEffortVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
