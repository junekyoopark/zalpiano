import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.subscription_person_pose = self.create_subscription(
            PoseStamped,
            '/person_pose',
            self.listener_callback_person_pose,
            10
        )
        self.subscription_odometry = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback_odometry,
            10
        )
        self.subscription_flag = self.create_subscription(
            String,
            '/direction_flag',
            self.listener_callback_flag,
            10
        )
        self.subscription_away_goal_pose = self.create_subscription(
            PoseStamped,
            '/away_goal_pose',
            self.listener_callback_away_goal_pose,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        self.current_flag = "toward"  # Default behavior
        self.current_odometry_pose = None
        self.latest_person_pose = None
        self.away_goal_pose = None

    def listener_callback_person_pose(self, msg):
        """
        Store the latest person pose and publish the goal pose based on the current flag.
        """
        self.latest_person_pose = msg
        self.publish_goal_pose()

    def listener_callback_odometry(self, msg):
        """
        Update the current odometry pose.
        """
        self.current_odometry_pose = msg.pose.pose

    def listener_callback_flag(self, msg):
        """
        Update the movement direction flag and publish the goal pose if applicable.
        """
        self.current_flag = msg.data
        self.publish_goal_pose()

    def listener_callback_away_goal_pose(self,msg):
        self.away_goal_pose = msg
        self.publish_goal_pose()

    def publish_goal_pose(self):
        """
        Publishes the goal pose based on the current flag and available data.
        """
        if self.latest_person_pose and self.current_odometry_pose:
            if self.current_flag == "toward":
                self.publisher.publish(self.latest_person_pose)
            elif self.current_flag == "away":
                self.publisher.publish(self.away_goal_pose)

    # def reflect_pose(self, target_pose, reference_pose):
    #     """
    #     Reflects the target pose about the reference pose.
    #     """
    #     # Reflection calculation: Reflecting a point across another point in 2D
    #     reflected_x = 2 * reference_pose.position.x - target_pose.position.x
    #     reflected_y = 2 * reference_pose.position.y - target_pose.position.y
    #     reflected_z = target_pose.position.z  # Maintain same Z

    #     # Construct the new reflected pose
    #     reflected_pose = target_pose
    #     reflected_pose.position.x = reflected_x
    #     reflected_pose.position.y = reflected_y
    #     reflected_pose.position.z = reflected_z
    #     return reflected_pose

def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        goal_pose_publisher.get_logger().info('Goal pose publisher node stopped cleanly')
    finally:
        goal_pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
