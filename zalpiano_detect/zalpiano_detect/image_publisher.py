import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Define the QoS profile
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile)
        # Attempt to open the camera using the MJPEG format
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        self.cap.set(cv2.CAP_PROP_FPS, 100)
        
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01, self.timer_callback)  # Adjust the timer to match the desired frame rate

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to a ROS Image message using CvBridge
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().warn('Camera frame capture failed')

    def on_shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    camera_publisher.on_shutdown()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
