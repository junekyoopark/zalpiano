import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('e_stop_button')
        # Subscriber for the joystick
        self.subscription = self.create_subscription(
            Joy,
            '/joy_e_stop',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for e_stop
        self.publisher_ = self.create_publisher(Twist, '/e_stop', 10)

    def joy_callback(self, msg):
        if msg.buttons[0] == 0:
            self.get_logger().info('Emergency Stop Engaged')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    emergency_stop = EmergencyStop()
    rclpy.spin(emergency_stop)
    # Destroy the node explicitly
    emergency_stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
