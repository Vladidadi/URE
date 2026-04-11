

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
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoders = [0, 0, 0, 0]
        self.last_time = self.get_clock().now()
        self.odom_lock = Lock()
        
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
            f'publish_tf={self.publish_tf})'
        )
    
    def checksum(self, data):
        """Calculate XOR checksum"""
        result = 0
        for byte in data:
            result ^= byte
        return result
    
    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor speeds and send to Arduino"""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Mecanum inverse kinematics
        lx = self.wheel_base_length / 2
        ly = self.wheel_base_width / 2
        
        v_fl = vx - vy - (lx + ly) * wz
        v_fr = vx + vy + (lx + ly) * wz
        v_rl = vx + vy - (lx + ly) * wz
        v_rr = vx - vy + (lx + ly) * wz
        
        # Convert m/s to pulses per 10ms
        factor = (self.encoder_cpr * 0.01) / (2 * math.pi * self.wheel_radius)
        
        # Limit motor speeds to reduce current draw (adjust as needed)
        max_speed = 30  # Reduce from 50 to limit current
        
        # Apply motor direction corrections based on actual hardware
        # Adjust these signs until forward motion works correctly
        m1 = max(-max_speed, min(max_speed, int(-v_fl * factor)))    # Front Left
        m2 = max(-max_speed, min(max_speed, int(-v_fr * factor)))    # Front Right (removed negation)
        m3 = max(-max_speed, min(max_speed, int(-v_rl * factor)))    # Rear Left (removed negation)
        m4 = max(-max_speed, min(max_speed, int(-v_rr * factor)))    # Rear Right
        
        self.send_motor_speeds(m1, m2, m3, m4)
    
    def send_motor_speeds(self, m1, m2, m3, m4):
        self.get_logger().info('sending motor speeds to arduino')

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
            self.serial.write(packet)
            self.serial.flush()
        except Exception as e:
            print(f"FAILED WRITING SERIAL")
            self.get_logger().error(f'Send failed: {e}')
    
    def read_encoders(self):
        """Read encoder values from Arduino"""
        # Build request: START | CMD | checksum | END
        data = [self.CMD_GET_ENCODERS]
        chk = self.checksum(data)
        packet = bytes([self.START_BYTE] + data + [chk, self.END_BYTE])
        
        try:
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush()
            
            response = self.serial.read(20)
            
            if len(response) == 20 and response[0] == self.START_BYTE and response[19] == self.END_BYTE:
                # Verify checksum
                if self.checksum(response[1:18]) == response[18]:
                    # Extract encoders (little-endian int32)
                    encoders = []
                    for i in range(4):
                        offset = 2 + i * 4
                        value = struct.unpack('<i', response[offset:offset+4])[0]
                        encoders.append(value)
                    return encoders
            
            return self.last_encoders
            
        except Exception as e:
            self.get_logger().error(f'Read failed: {e}')
            return self.last_encoders
    
    def update_odometry(self):
        """Read encoders and update odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Read encoders
        encoders = self.read_encoders()
        
        # Calculate deltas
        deltas = [encoders[i] - self.last_encoders[i] for i in range(4)]
        self.last_encoders = encoders
        
        # Convert pulses to radians for joint states
        for i in range(4):
            delta_radians = (deltas[i] / self.encoder_cpr) * 2 * math.pi
            self.wheel_positions[i] += delta_radians
            # Normalize to [-pi, pi]
            self.wheel_positions[i] = math.atan2(math.sin(self.wheel_positions[i]), 
                                                   math.cos(self.wheel_positions[i]))
            # Calculate velocity (rad/s)
            if dt > 0:
                self.wheel_velocities[i] = delta_radians / dt
        
        # Publish joint states
        self.publish_joint_states(current_time)
        
        # Convert pulses to meters
        wheel_dist = [(d / self.encoder_cpr) * (2 * math.pi * self.wheel_radius) for d in deltas]
        
        # Apply motor direction corrections (must match cmd_vel signs)
        d_fl = -wheel_dist[0]
        d_fr = -wheel_dist[1]  # Removed negation
        d_rl = -wheel_dist[2]  # Removed negation
        d_rr = -wheel_dist[3]
        
        # Forward kinematics
        lx = self.wheel_base_length / 2
        ly = self.wheel_base_width / 2
        
        vx = (d_fl + d_fr + d_rl + d_rr) / 4.0
        vy = (-d_fl + d_fr + d_rl - d_rr) / 4.0
        w = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * (lx + ly))
        
        # Update pose
        with self.odom_lock:
            dx = vx * math.cos(self.theta) - vy * math.sin(self.theta)
            dy = vx * math.sin(self.theta) + vy * math.cos(self.theta)
            
            self.x += dx
            self.y += dy
            self.theta += w
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish
        if dt > 0:
            self.publish_odom(current_time, vx/dt, vy/dt, w/dt)
    
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

