import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, OutputDevice

class DualMotorL298NNode(Node):
    def __init__(self):
        super().__init__('dual_motor_l298n_node')
        
        # L298N Motor Driver Pinout (based on your configuration)
        # Motor A (Right Motor)
        self.motor_a_pwm = PWMOutputDevice(12, frequency=1000)  # PWM_0 (GPIO 12, physical pin 32)
        self.motor_a_in1 = OutputDevice(22)  # IN1 (physical pin 15)
        self.motor_a_in2 = OutputDevice(27)  # IN2 (physical pin 13)
        
        # Motor B (Left Motor) 
        self.motor_b_pwm = PWMOutputDevice(13, frequency=1000)  # PWM_1 (GPIO 13, physical pin 33)
        self.motor_b_in3 = OutputDevice(17)  # IN3 (physical pin 11)
        self.motor_b_in4 = OutputDevice(18)  # IN4 (physical pin 12)
        
        # Subscribe to Twist commands (common for robot control)
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        # Subscribe to individual motor speeds
        self.motor_a_subscription = self.create_subscription(
            Float32,
            'motor_a_speed',
            self.motor_a_callback,
            10)
            
        self.motor_b_subscription = self.create_subscription(
            Float32,
            'motor_b_speed',
            self.motor_b_callback,
            10)
        
        self.get_logger().info('Dual Motor L298N Node started')
        self.get_logger().info('Motor A: PWM=GPIO12, IN1=GPIO22, IN2=GPIO27')
        self.get_logger().info('Motor B: PWM=GPIO13, IN3=GPIO17, IN4=GPIO18')
    
    def set_motor_a(self, speed):
        """
        Set Motor A speed and direction
        speed: -1.0 to 1.0 (negative = reverse, positive = forward)
        """
        abs_speed = min(1.0, abs(speed))
        
        if speed > 0:  # Forward
            self.motor_a_in1.on()
            self.motor_a_in2.off()
        elif speed < 0:  # Reverse
            self.motor_a_in1.off()
            self.motor_a_in2.on()
        else:  # Stop
            self.motor_a_in1.off()
            self.motor_a_in2.off()
        
        self.motor_a_pwm.value = abs_speed
        self.get_logger().info(f'Motor A speed: {speed:.2f}')
    
    def set_motor_b(self, speed):
        """
        Set Motor B speed and direction
        speed: -1.0 to 1.0 (negative = reverse, positive = forward)
        """
        abs_speed = min(1.0, abs(speed))
        
        if speed > 0:  # Forward
            self.motor_b_in3.on()
            self.motor_b_in4.off()
        elif speed < 0:  # Reverse
            self.motor_b_in3.off()
            self.motor_b_in4.on()
        else:  # Stop
            self.motor_b_in3.off()
            self.motor_b_in4.off()
        
        self.motor_b_pwm.value = abs_speed
        self.get_logger().info(f'Motor B speed: {speed:.2f}')
    
    def motor_a_callback(self, msg):
        """Handle individual Motor A speed commands"""
        speed = max(-1.0, min(1.0, msg.data))
        self.set_motor_a(speed)
    
    def motor_b_callback(self, msg):
        """Handle individual Motor B speed commands"""
        speed = max(-1.0, min(1.0, msg.data))
        self.set_motor_b(speed)
    
    def twist_callback(self, msg):
        """
        Handle Twist messages for differential drive
        Converts linear and angular velocity to left/right motor speeds
        """
        linear = msg.linear.x   # Forward/backward
        angular = msg.angular.z # Rotation
        
        # Simple differential drive calculation
        # Adjust these values based on your robot's wheel separation
        wheel_separation = 0.2  # meters between wheels (adjust for your robot)
        
        # Calculate left and right wheel speeds
        left_speed = linear + (angular * wheel_separation / 2.0)
        right_speed = linear - (angular * wheel_separation / 2.0)
        
        # Normalize speeds to [-1.0, 1.0]
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
        
        self.set_motor_a(right_speed)  # Motor A = Right
        self.set_motor_b(left_speed)   # Motor B = Left
    
    def stop_all_motors(self):
        """Emergency stop - turn off all motors"""
        self.set_motor_a(0)
        self.set_motor_b(0)
        self.get_logger().info('All motors stopped')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_all_motors()
        self.motor_a_pwm.close()
        self.motor_b_pwm.close()
        self.motor_a_in1.close()
        self.motor_a_in2.close()
        self.motor_b_in3.close()
        self.motor_b_in4.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorL298NNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
