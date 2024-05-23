import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped  # Modified import
from geometry_msgs.msg import Quaternion

import tf2_ros
from geometry_msgs.msg import TransformStamped

from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

# import tf_transformations

W, H = 1280, 800
fov_horizontal = 1.2217  # radians, approximately 70 degrees

# Calculate fx and fy using the horizontal FoV and assuming square pixels
fx = fy = W / (2 * np.tan(fov_horizontal / 2))

# The optical center is at the center of the image
cx, cy = W / 2, H / 2

# Your distortion coefficients
k1, k2, p1, p2, k3 = -0.25, 0.12, -0.00028, -0.00005, 0.0

# Known Z depth
constant_z = 2.00  # Assuming a constant Z value for the demonstration

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3])

        # Initialize the camera with specific settings
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        self.cap.set(cv2.CAP_PROP_FPS, 100)

        self.bridge = CvBridge()
        
        # Pose publisher for each marker ID
        self.pose_publishers = {}
        for marker_id in range(4):  # IDs 0 to 3
            topic_name = f'/aruco_pose_cov/marker_{marker_id}'
            publisher = self.create_publisher(PoseWithCovarianceStamped, topic_name, 10)
            self.pose_publishers[marker_id] = publisher

        # Create a timer to process frames at a fixed interval (e.g., 10 Hz)
        # Needed for rclpy spin
        self.timer = self.create_timer(0.01, self.process_frames)


        self.person_pose_publisher = self.create_publisher(PoseStamped, '/person_pose', 10)


    def detect_yellow_region(self, frame):
        """
        Detect the largest yellow region in the frame and return its centroid and area.
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for yellow color and threshold
        lower_yellow = np.array([20, 100, 100])  # Lower boundary for yellow
        upper_yellow = np.array([30, 255, 255])  # Upper boundary for yellow
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return cx, cy, cv2.contourArea(largest_contour)
        return None

    def image_point_to_world(self, x, y):
        """
        Convert 2D image points to 3D world coordinates with a known Z depth.
        """
        X = (x - cx) * constant_z / fx
        Y = -((y - cy) * constant_z / fy)
        return X, Y, constant_z

    def process_frames(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if 0 <= marker_id <= 3:  # Check if the marker ID is between 0 and 3
                        corner = corners[i]
                        aruco.drawDetectedMarkers(frame, [corner])

                        c = corner[0]
                        center = c.mean(axis=0)
                        X, Y, Z = self.image_point_to_world(center[0], center[1])

                        vector = c[1] - c[0]
                        angle = np.arctan2(vector[0], vector[1])
                        cy = np.cos(angle * 0.5)
                        sy = np.sin(angle * 0.5)

                        pose_cov_msg = PoseWithCovarianceStamped()
                        pose_cov_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_cov_msg.header.frame_id = "map"
                        pose_cov_msg.pose.pose.position.x = -X
                        pose_cov_msg.pose.pose.position.y = -Y
                        pose_cov_msg.pose.pose.position.z = Z # Adjust Z as before
                        pose_cov_msg.pose.pose.orientation.x = 0.0
                        pose_cov_msg.pose.pose.orientation.y = 0.0
                        pose_cov_msg.pose.pose.orientation.z = -cy
                        pose_cov_msg.pose.pose.orientation.w = sy

                        pose_cov_msg.pose.covariance = [0.0] * 36  # Initialize covariance as zero (adjust as needed)

                        if marker_id in self.pose_publishers:
                            self.pose_publishers[marker_id].publish(pose_cov_msg)

                        position_text = f"3D Pos: ({X:.2f}, {Y:.2f}, {cy:.2f})"
                        cv2.putText(frame, position_text, (int(center[0] + 20), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            yellow_centroid = self.detect_yellow_region(frame)
            if yellow_centroid:
                cx, cy, area = yellow_centroid
                X, Y, Z = self.image_point_to_world(cx, cy)

                # Prepare the pose message
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.pose.position.x = -X
                pose_msg.pose.pose.position.y = -Y
                pose_msg.pose.pose.position.z = Z
                pose_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Neutral orientation
                pose_msg.pose.covariance = [0.0] * 36  # Initialize covariance as zero (adjust as needed)

                # Publish the pose
                self.person_pose_publisher.publish(pose_msg)

                # Optional: draw the centroid on the frame for visual feedback
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)


            cv2.imshow("ArUco Marker Detection and 3D Position", frame)
            cv2.waitKey(1) 
    
def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
