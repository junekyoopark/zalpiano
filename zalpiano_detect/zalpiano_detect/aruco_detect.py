#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters_create()

        # Load camera parameters
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3])

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Initialize the camera
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        self.cap.set(cv2.CAP_PROP_FPS, 100)

        # Create a timer to process frames at a fixed interval
        self.timer = self.create_timer(0.01, self.process_frames)

    def process_frames(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs)
                
                for i, marker_id in enumerate(ids.flatten()):
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_depth_frame'
                    t.child_frame_id = f'aruco_marker_{marker_id}'
                    
                    t.transform.translation.x = tvecs[i][0][0]
                    t.transform.translation.y = tvecs[i][0][1]
                    t.transform.translation.z = tvecs[i][0][2]

                    # Convert the rotation vector to quaternion
                    r = cv2.Rodrigues(rvecs[i][0])[0]
                    t.transform.rotation.x = r[0]
                    t.transform.rotation.y = r[1]
                    t.transform.rotation.z = r[2]
                    t.transform.rotation.w = r[3]
                    
                    self.tfbroadcaster.sendTransform(t)
                    cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1) # Visualize axes

            cv2.imshow("ArUco Marker Detection", frame)
            cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
