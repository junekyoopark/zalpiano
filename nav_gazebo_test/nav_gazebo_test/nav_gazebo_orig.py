import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped  # Modified import
from geometry_msgs.msg import Quaternion

import tf2_ros
from geometry_msgs.msg import TransformStamped

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
constant_z = 4.85  # Assuming a constant Z value for the demonstration

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3])

        # Pose publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/aruco_pose_cov', 10)
        # self.br = tf2_ros.TransformBroadcaster(self)

    # def broadcast_transform(self, X, Y, Z, angle, frame_id="map", child_frame_id="aruco_marker_link"):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = frame_id
    #     t.child_frame_id = child_frame_id
        
    #     t.transform.translation.x = X
    #     t.transform.translation.y = Y
    #     t.transform.translation.z = Z - 4.85  # Adjusting Z based on your code
        
    #     cy = np.cos(angle * 0.5)
    #     sy = np.sin(angle * 0.5)
        
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = sy
    #     t.transform.rotation.w = cy
        
    #     self.br.sendTransform(t)

    def image_point_to_world(self, x, y):
        """
        Convert 2D image points to 3D world coordinates with a known Z depth.
        """
        X = (x - cx) * constant_z / fx
        Y = -((y - cy) * constant_z / fy)
        return X, Y, constant_z

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Undistort the image
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and 0 in ids:
            index_of_id_0 = np.where(ids == 0)[0]
            corners_of_id_0 = [corners[i] for i in index_of_id_0]
            aruco.drawDetectedMarkers(frame_undistorted, corners_of_id_0)

            for corner in corners_of_id_0:
                c = corner[0]
                center = c.mean(axis=0)
                X, Y, Z = self.image_point_to_world(center[0], center[1])

                vector = c[1] - c[0]
                angle = np.arctan2(vector[1], vector[0])
                cy = np.cos(angle * 0.5)
                sy = np.sin(angle * 0.5)

                # Prepare PoseWithCovarianceStamped message instead of PoseStamped
                pose_cov_msg = PoseWithCovarianceStamped()
                pose_cov_msg.header.stamp = self.get_clock().now().to_msg()
                print(pose_cov_msg.header.stamp)
                pose_cov_msg.header.frame_id = "map"
                pose_cov_msg.pose.pose.position.x = X 
                pose_cov_msg.pose.pose.position.y = Y
                pose_cov_msg.pose.pose.position.z = Z - 4.85  # Adjust Z as before
                pose_cov_msg.pose.pose.orientation.x = 0.0
                pose_cov_msg.pose.pose.orientation.y = 0.0
                pose_cov_msg.pose.pose.orientation.z = cy
                pose_cov_msg.pose.pose.orientation.w = sy


                # For covariance, you might initialize it as follows if you don't compute it specifically
                # This example sets all values to 0, which means unknown uncertainty; adjust as needed
                pose_cov_msg.pose.covariance = [0.0] * 36  # 6x6 matrix flattened into an array

                self.pose_publisher.publish(pose_cov_msg)

                # self.broadcast_transform(X, Y, Z, angle)

                position_text = f"3D Pos: ({X:.2f}, {Y:.2f}, {Z:.2f})"
                cv2.putText(frame_undistorted, position_text, (int(center[0] + 20), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Display the undistorted frame with the ArUco marker and 3D position
        cv2.imshow("ArUco Marker Detection and 3D Position", frame_undistorted)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
