import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Point
import tf_transformations

class CentroidFollower(Node):
    def __init__(self):
        super().__init__('centroid_follower')
        self.get_logger().info("Centroid follower starting...")

        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('angle_scale', 0.5)

        self.image_width = self.get_parameter('image_width').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.angle_scale = self.get_parameter('angle_scale').value

        # Store latest odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Subscriptions
        self.create_subscription(Point , '/object_centroid', self.callback_centroid, 10)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 10)

        # Publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("Centroid follower ready: listening to /object_centroid and /odom")

    def callback_odom(self, msg: Odometry):
        """Update robot position and yaw from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion → yaw
        self.robot_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                                    1 - 2 * (q.y*q.y + q.z*q.z))

    def callback_centroid(self, msg: PoseStamped):
        """Compute a goal point in odom frame based on object centroid offset."""
        cx = msg.x
        # --- Compute yaw offset from centroid position ---
# cx = msg.pose.position.x
        offset = (cx - self.image_width / 2.0) / (self.image_width / 2.0)
        target_yaw = self.robot_yaw - offset * self.angle_scale

        # --- Compute 1 m lookahead in robot frame (robot yaw, not target_yaw!) ---
        # dx_robot = self.lookahead
        # dy_robot = 0.0
        dx_robot = self.lookahead * math.cos(target_yaw)
        dy_robot = self.lookahead * math.sin(target_yaw)
        # Rotate this vector into odom/map frame using the *robot's* current yaw
        # dx_odom = math.cos(self.robot_yaw) * dx_robot - math.sin(self.robot_yaw) * dy_robot
        # dy_odom = math.sin(self.robot_yaw) * dx_robot + math.cos(self.robot_yaw) * dy_robot

        # goal_x = self.robot_x + dx_odom
        # goal_y = self.robot_y + dy_odom
        goal_x = self.robot_x+dx_robot
        goal_y = self.robot_y+dy_robot
        # offset = (cx - self.image_width / 2.0) / (self.image_width / 2.0)
        # target_yaw = self.robot_yaw - offset * self.angle_scale

        # # Compute lookahead goal in  robot frame


        # # Rotate into odom frame
        # dx_odom = math.cos(self.robot_yaw) * dx_robot - math.sin(self.robot_yaw) * dy_robot
        # dy_odom = math.sin(self.robot_yaw) * dx_robot + math.cos(self.robot_yaw) * dy_robot

        # # Add to robot position
        # goal_x = self.robot_x + dx_odom
        # goal_y = self.robot_y + dy_odom

        # Build PoseStamped goal in odom frame
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, target_yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.publisher.publish(goal)
        self.get_logger().info(f"Published goal: ({goal_x:.2f}, {goal_y:.2f}) yaw={math.degrees(target_yaw):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = CentroidFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
