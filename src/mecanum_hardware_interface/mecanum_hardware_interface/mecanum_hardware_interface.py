

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import struct
import math
import time
import serial
from threading import Lock
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class MecanumHardwareInterface(Node):
    START_BYTE = 0xAA
    END_BYTE = 0x55
    CMD_SET_SPEED = 0x01
    CMD_GET_ENCODERS = 0x02
    
    def __init__(self):
        super().__init__('mecanum_hardware_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_width', 0.20)
        self.declare_parameter('wheel_base_length', 0.20)
        self.declare_parameter('encoder_cpr', 5764)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # When robot_localization EKF runs with publish_tf: true, set publish_tf false in
        # mecanum_params.yaml to avoid two publishers for odom -> base_link.
        # Default True so /odom TF still exists if the node is run without that yaml or without EKF.
        self.declare_parameter('publish_tf', True)
        # Ignore per-tick encoder noise (counts); helps stationary odom jitter.
        self.declare_parameter('encoder_deadband', 8)
        # Reject a tick if any wheel delta exceeds this (counts); catches bad reads / wrap glitches.
        self.declare_parameter('max_encoder_delta_per_tick', 4000)
        # Drop tiny cmd_vel so motors (and encoders) are not nudged by teleop noise / quantization.
        self.declare_parameter('cmd_vel_linear_deadband', 0.02)
        self.declare_parameter('cmd_vel_angular_deadband', 0.05)
        # Per-wheel command sent to Arduino (same units as legacy hardcoded 30; lower = slower).
        self.declare_parameter('max_motor_command', 22)

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_width = self.get_parameter('wheel_base_width').value
        self.wheel_base_length = self.get_parameter('wheel_base_length').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.encoder_deadband = int(self.get_parameter('encoder_deadband').value)
        self.max_encoder_delta_per_tick = int(
            self.get_parameter('max_encoder_delta_per_tick').value
        )
        self.cmd_vel_linear_deadband = float(
            self.get_parameter('cmd_vel_linear_deadband').value
        )
        self.cmd_vel_angular_deadband = float(
            self.get_parameter('cmd_vel_angular_deadband').value
        )
        self.max_motor_command = max(
            1, min(255, int(self.get_parameter('max_motor_command').value))
        )

        self.odom_lock = Lock()
        self._serial_lock = Lock()

        # Connect to Arduino
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
            
            # Wait for READY
            for _ in range(50):
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore')
                    if 'READY' in line:
                        self.get_logger().info('Arduino ready!')
                        break
                time.sleep(0.1)
            # Drop any trailing text/newlines so binary encoder frames stay aligned.
            self.serial.reset_input_buffer()
            self.send_motor_speeds(0, 0, 0, 0)
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoders = [0, 0, 0, 0]
        self._encoder_baseline_done = False
        self.last_time = self.get_clock().now()
        
        # Joint states (wheel positions in radians)
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # ROS2 interfaces
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Low voltage protection
        self.low_voltage_threshold = 6.0  # Stop motors below this voltage
        self.last_voltage = 8.0
        
        # Timer for odometry updates
        self.create_timer(0.05, self.update_odometry)  # 20Hz
        
        self.get_logger().info(
            f'Mecanum Hardware Interface ready '
            f'(odom_frame={self.odom_frame}, base_frame={self.base_frame}, '
            f'publish_tf={self.publish_tf}, max_motor_command={self.max_motor_command})'
        )
    
    def checksum(self, data):
        """Calculate XOR checksum"""
        result = 0
        for byte in data:
            result ^= byte
        return result

    @staticmethod
    def _encoder_delta_int32(new_count, old_count):
        """Signed delta with int32 wrap (Arduino encoders are int32)."""
        a = new_count & 0xFFFFFFFF
        b = old_count & 0xFFFFFFFF
        d = (a - b) & 0xFFFFFFFF
        if d >= 0x80000000:
            d -= 0x100000000
        return int(d)

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor speeds and send to Arduino"""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        lb = self.cmd_vel_linear_deadband
        ab = self.cmd_vel_angular_deadband
        if abs(vx) < lb:
            vx = 0.0
        if abs(vy) < lb:
            vy = 0.0
        if abs(wz) < ab:
            wz = 0.0

        # Mecanum inverse kinematics
        lx = self.wheel_base_length / 2
        ly = self.wheel_base_width / 2
        
        v_fl = vx - vy - (lx + ly) * wz
        v_fr = vx + vy + (lx + ly) * wz
        v_rl = vx + vy - (lx + ly) * wz
        v_rr = vx - vy + (lx + ly) * wz
        
        # Convert m/s to pulses per 10ms
        factor = (self.encoder_cpr * 0.01) / (2 * math.pi * self.wheel_radius)
        
        lim = self.max_motor_command
        m1 = max(-lim, min(lim, int(-v_fl * factor)))
        m2 = max(-lim, min(lim, int(-v_fr * factor)))
        m3 = max(-lim, min(lim, int(-v_rl * factor)))
        m4 = max(-lim, min(lim, int(-v_rr * factor)))
        
        self.send_motor_speeds(m1, m2, m3, m4)
    
    def send_motor_speeds(self, m1, m2, m3, m4):
        """Send motor speeds to Arduino"""
        # Convert signed to unsigned bytes
        bytes_list = []
        for m in [m1, m2, m3, m4]:
            bytes_list.append(m if m >= 0 else 256 + m)
        
        # Build packet: START | CMD | m1 | m2 | m3 | m4 | checksum | END
        data = [self.CMD_SET_SPEED] + bytes_list
        chk = self.checksum(data)
        packet = bytes([self.START_BYTE] + data + [chk, self.END_BYTE])
        
        try:
            with self._serial_lock:
                self.serial.write(packet)
                self.serial.flush()
        except Exception as e:
            self.get_logger().error(f'Send failed: {e}')
    
    def read_encoders(self):
        """Read encoder values from Arduino (resync on 0xAA; serial access serialized)."""
        data = [self.CMD_GET_ENCODERS]
        chk = self.checksum(data)
        packet = bytes([self.START_BYTE] + data + [chk, self.END_BYTE])

        try:
            with self._serial_lock:
                self.serial.write(packet)
                self.serial.flush()

                deadline = time.monotonic() + 0.15
                while time.monotonic() < deadline:
                    b = self.serial.read(1)
                    if not b:
                        continue
                    if b[0] != self.START_BYTE:
                        continue
                    rest = self.serial.read(19)
                    if len(rest) != 19:
                        continue
                    response = bytes([self.START_BYTE]) + rest
                    if response[19] != self.END_BYTE:
                        continue
                    if self.checksum(response[1:18]) != response[18]:
                        continue
                    encoders = []
                    for i in range(4):
                        offset = 2 + i * 4
                        value = struct.unpack('<i', response[offset:offset+4])[0]
                        encoders.append(value)
                    return encoders
                return None

        except Exception as e:
            self.get_logger().error(f'Read failed: {e}')
            return None
    
    def update_odometry(self):
        """Read encoders and update odometry.

        Always publishes /odom, odom->base_link TF (if enabled), and /joint_states after the
        encoder baseline is established, even when a serial read fails. That keeps TF from
        expiring in RViz (no more blinking odom).
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        def publish_hold_stationary():
            self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
            self.publish_joint_states(current_time)
            self.publish_odom(current_time, 0.0, 0.0, 0.0)

        encoders = self.read_encoders()
        if encoders is None:
            if self._encoder_baseline_done:
                publish_hold_stationary()
            self.last_time = current_time
            return

        if not self._encoder_baseline_done:
            self.last_encoders = list(encoders)
            self._encoder_baseline_done = True
            publish_hold_stationary()
            self.last_time = current_time
            return

        valid_dt = 0.0 < dt <= 2.0
        if not valid_dt:
            if dt > 2.0:
                self.get_logger().warn(f'Bad dt={dt}, holding pose (no integration)')
            publish_hold_stationary()
            self.last_time = current_time
            return

        raw_deltas = [
            self._encoder_delta_int32(encoders[i], self.last_encoders[i]) for i in range(4)
        ]
        db = self.encoder_deadband
        deltas = [0 if abs(d) <= db else d for d in raw_deltas]

        if any(abs(d) > self.max_encoder_delta_per_tick for d in raw_deltas):
            self.get_logger().warn(
                f'Rejecting encoder tick (outlier): raw_deltas={raw_deltas}'
            )
            publish_hold_stationary()
            self.last_time = current_time
            return

        self.last_encoders = encoders

        for i in range(4):
            delta_radians = (deltas[i] / self.encoder_cpr) * 2 * math.pi
            self.wheel_positions[i] += delta_radians
            self.wheel_velocities[i] = delta_radians / dt

        wheel_dist = [(d / self.encoder_cpr) * (2 * math.pi * self.wheel_radius) for d in deltas]

        d_fl = -wheel_dist[0]
        d_fr = -wheel_dist[1]
        d_rl = -wheel_dist[2]
        d_rr = -wheel_dist[3]

        lx = self.wheel_base_length / 2
        ly = self.wheel_base_width / 2

        vx = (d_fl + d_fr + d_rl + d_rr) / 4.0
        vy = (-d_fl + d_fr + d_rl - d_rr) / 4.0
        w = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * (lx + ly))

        with self.odom_lock:
            dx = vx * math.cos(self.theta) - vy * math.sin(self.theta)
            dy = vx * math.sin(self.theta) + vy * math.cos(self.theta)

            self.x += dx
            self.y += dy
            self.theta += w
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_joint_states(current_time)
        self.publish_odom(current_time, vx / dt, vy / dt, w / dt)
        self.last_time = current_time

    def publish_joint_states(self, stamp):
        """Publish joint states for robot_state_publisher"""
        joint_state = JointState()
        joint_state.header.stamp = stamp.to_msg()
        joint_state.header.frame_id = self.base_frame
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_state.position = self.wheel_positions
        joint_state.velocity = self.wheel_velocities
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_odom(self, stamp, vx, vy, w):
        """Publish odometry and TF"""
        with self.odom_lock:
            odom = Odometry()
            odom.header.stamp = stamp.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            qz = math.sin(self.theta / 2)
            qw = math.cos(self.theta / 2)
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = w
            
            self.odom_pub.publish(odom)

            if self.publish_tf and self.tf_broadcaster is not None:
                t = TransformStamped()
                t.header.stamp = stamp.to_msg()
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(t)

   
    def destroy_node(self):
        """Stop motors on shutdown"""
        try:
            self.send_motor_speeds(0, 0, 0, 0)
            self.serial.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumHardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

