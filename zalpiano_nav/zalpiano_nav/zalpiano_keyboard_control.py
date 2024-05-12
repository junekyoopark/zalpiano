import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.create_publisher(Twist, '/zalpiano_base_controller/cmd_vel_unstamped', qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer frequency could be adjusted for responsiveness
        self.get_logger().info('Keyboard Control Node has started.')

    def timer_callback(self):
        cv2.imshow('Press "WASD" to control, "Q" to quit.', 255 * np.ones((100, 400, 3), np.uint8))
        key = cv2.waitKey(10) & 0xFF  # Waiting a bit longer to ensure key press capture

        msg = Twist()

        if key == ord('w'):
            msg.linear.x = 1.0
        elif key == ord('s'):
            msg.linear.x = -1.0
        elif key == ord('a'):
            msg.angular.z = 1.0
        elif key == ord('d'):
            msg.angular.z = -1.0
        elif key == ord('q'):
            self.shutdown_node()

        if key in [ord('w'), ord('s'), ord('a'), ord('d')]:
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg}')

    def shutdown_node(self):
        self.get_logger().info('Shutting down keyboard control node.')
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control_node = KeyboardControlNode()
    try:
        rclpy.spin(keyboard_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
