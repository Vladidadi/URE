#!/usr/bin/env python3
"""Ideal mecanum kinematic simulator: cmd_vel -> /odom + /joint_states (+ optional /scan, /imu)."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster


class MecanumKinematicSim(Node):
    JOINT_NAMES = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    ]

    def __init__(self):
        super().__init__('mecanum_kinematic_sim')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_width', 0.20)
        self.declare_parameter('wheel_base_length', 0.20)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_fake_scan', False)
        self.declare_parameter('publish_fake_imu', False)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('scan_range', 25.0)
        self.declare_parameter('scan_beams', 360)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base_width = float(self.get_parameter('wheel_base_width').value)
        self.wheel_base_length = float(self.get_parameter('wheel_base_length').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_fake_scan = bool(self.get_parameter('publish_fake_scan').value)
        self.publish_fake_imu = bool(self.get_parameter('publish_fake_imu').value)
        self.laser_frame = self.get_parameter('laser_frame').value
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.scan_range = float(self.get_parameter('scan_range').value)
        self.scan_beams = int(self.get_parameter('scan_beams').value)

        self._cmd = Twist()
        self._cmd_stamp = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self._pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self._pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self._tf_bc = TransformBroadcaster(self) if self.publish_tf else None
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_scan = (
            self.create_publisher(LaserScan, 'scan', scan_qos)
            if self.publish_fake_scan
            else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, '/imu/mpu6050', 10) if self.publish_fake_imu else None
        )

        self._period = 0.05
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f'Kinematic sim: {self.odom_frame}->{self.base_frame}, '
            f'fake_scan={self.publish_fake_scan}, fake_imu={self.publish_fake_imu}, '
            f'publish_tf={self.publish_tf}'
        )

    def _cmd_cb(self, msg: Twist):
        self._cmd = msg
        self._cmd_stamp = self.get_clock().now()

    def _body_vel(self):
        now = self.get_clock().now()
        if (now - self._cmd_stamp).nanoseconds / 1e9 > self.cmd_timeout_sec:
            return 0.0, 0.0, 0.0
        return self._cmd.linear.x, self._cmd.linear.y, self._cmd.angular.z

    def _tick(self):
        stamp = self.get_clock().now()
        dt = self._period
        vx, vy, wz = self._body_vel()

        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        lx = self.wheel_base_length / 2.0
        ly = self.wheel_base_width / 2.0
        v_fl = vx - vy - (lx + ly) * wz
        v_fr = vx + vy + (lx + ly) * wz
        v_rl = vx + vy - (lx + ly) * wz
        v_rr = vx - vy + (lx + ly) * wz

        # Match hardware driver sign convention for wheel angular velocity (rad/s)
        om_fl = -v_fl / self.wheel_radius
        om_fr = -v_fr / self.wheel_radius
        om_rl = -v_rl / self.wheel_radius
        om_rr = -v_rr / self.wheel_radius
        oms = [om_fl, om_fr, om_rl, om_rr]
        for i in range(4):
            self.wheel_velocities[i] = oms[i]
            self.wheel_positions[i] += oms[i] * dt

        self._publish_joint_states(stamp)
        self._publish_odom(stamp, vx, vy, wz)
        if self._pub_scan is not None:
            # Slightly later stamp helps AMCL/costmaps sync laser with TF cache
            self._publish_scan(self.get_clock().now())
        if self._pub_imu is not None:
            self._publish_imu(stamp, wz)

    def _publish_joint_states(self, stamp):
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.header.frame_id = self.base_frame
        js.name = self.JOINT_NAMES
        js.position = list(self.wheel_positions)
        js.velocity = list(self.wheel_velocities)
        self._pub_js.publish(js)

    def _publish_odom(self, stamp, vx, vy, wz):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._pub_odom.publish(odom)

        if self._tf_bc is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_bc.sendTransform(t)

    def _publish_scan(self, stamp):
        n = max(8, self.scan_beams)
        scan = LaserScan()
        scan.header.stamp = stamp.to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - (2.0 * math.pi / n)
        scan.angle_increment = 2.0 * math.pi / n
        scan.time_increment = 0.0
        scan.scan_time = self._period
        scan.range_min = 0.05
        scan.range_max = self.scan_range
        scan.ranges = [self.scan_range * 0.95] * n
        self._pub_scan.publish(scan)

    def _publish_imu(self, stamp, wz):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.base_frame
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = wz
        imu.linear_acceleration.z = 9.81
        imu.orientation_covariance[8] = 0.05
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[8] = 0.1
        self._pub_imu.publish(imu)


def main():
    rclpy.init()
    node = MecanumKinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
