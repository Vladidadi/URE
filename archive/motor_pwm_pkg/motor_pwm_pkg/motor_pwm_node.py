import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice

class MotorPWMNode(Node):
    def __init__(self):
        super().__init__('motor_pwm_node')
        
        # Initialize PWM on GPIO 18 (hardware PWM pin)
        self.motor_pwm = PWMOutputDevice(18, frequency=1000)  # 1kHz
        
        # Subscribe to motor speed commands (0.0 to 1.0)
        self.subscription = self.create_subscription(
            Float32,
            'motor_speed',
            self.speed_callback,
            10)
        
        self.get_logger().info('Motor PWM Node started on GPIO 18')
    
    def speed_callback(self, msg):
        # Clamp speed between 0.0 and 1.0
        speed = max(0.0, min(1.0, abs(msg.data)))
        self.motor_pwm.value = speed
        self.get_logger().info(f'Motor speed set to: {speed:.2f}')
    
    def destroy_node(self):
        # Clean shutdown
        self.motor_pwm.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorPWMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
