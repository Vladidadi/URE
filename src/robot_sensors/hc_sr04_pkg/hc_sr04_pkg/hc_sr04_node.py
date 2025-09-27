import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor
import time

class HCSR04Node(Node):
    def __init__(self):
        super().__init__('hc_sr04_node')
        
        # HC-SR04 connections (adjust GPIO pins as needed)
        # Trigger pin and Echo pin
        self.sensor = DistanceSensor(echo=10, trigger=9)  
        
        # Publisher for range data
        self.range_publisher = self.create_publisher(Range, 'ultrasonic_range', 10)
        
        # Timer for periodic sensor readings
        self.timer = self.create_timer(0.1, self.publish_range)  # 10Hz
        
        # Configure Range message parameters
        self.range_msg = Range()
        self.range_msg.header.frame_id = 'ultrasonic_link'
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.26  # ~15 degrees in radians
        self.range_msg.min_range = 0.02  # 2cm minimum range
        self.range_msg.max_range = 4.0   # 4m maximum range
        
        self.get_logger().info('HC-SR04 Ultrasonic Sensor Node started')
        self.get_logger().info(f'Trigger: GPIO9, Echo: GPIO10')
    
    def publish_range(self):
        try:
            # Get distance measurement
            distance = self.sensor.distance
            
            # Update message
            self.range_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Handle out-of-range readings
            if distance == float('inf') or distance > self.range_msg.max_range:
                self.range_msg.range = self.range_msg.max_range
            elif distance < self.range_msg.min_range:
                self.range_msg.range = self.range_msg.min_range
            else:
                self.range_msg.range = distance
            
            # Publish the range data
            self.range_publisher.publish(self.range_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error reading sensor: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'sensor'):
            self.sensor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HCSR04Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
