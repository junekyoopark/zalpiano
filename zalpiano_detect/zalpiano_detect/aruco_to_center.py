import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ArucoTransformer(Node):
    def __init__(self):
        super().__init__('aruco_transformer')
        rclpy.logging.get_logger('ArucoTransformer').info("Initializing ArucoTransformer")
        
        # Ensure parameters are declared and retrieved properly
        self.declare_parameter('marker_ids', [0, 1, 2, 3])
        marker_ids = self.get_parameter('marker_ids').get_parameter_value().integer_array_value
        rclpy.logging.get_logger('ArucoTransformer').info(f"Marker IDs: {marker_ids}")

        # Initialize subscriptions list
        self._subscriptions = []  # If this causes an error, it should be visible now
        self.marker_publishers = {}
        rclpy.logging.get_logger('ArucoTransformer').info("Subscriptions and Publishers initialized")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        for marker_id in marker_ids:
            topic_name = f'/aruco_pose_cov/marker_{marker_id}'
            subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                topic_name,
                lambda msg, marker_id=marker_id: self.handle_pose(msg, marker_id),
                10
            )
            self._subscriptions.append(subscription)
            rclpy.logging.get_logger('ArucoTransformer').info(f"Subscribed to {topic_name}")

            publisher = self.create_publisher(PoseWithCovarianceStamped, f'/aruco_center_{marker_id}', 10)
            self.marker_publishers[marker_id] = publisher
            self.get_logger().info(f"Publisher created for /aruco_center_{marker_id}")

    def handle_pose(self, msg, marker_id):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        # transform.header.frame_id = f'aruco_marker_{marker_id}_link'
        transform.child_frame_id = f'aruco_center_link{marker_id}'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        # Publish the pose to the corresponding topic
        self.marker_publishers[marker_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
