#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

class MockWindSensor(Node):
    """Simulates a hardware wind sensor by generating a synthetic vector field based on position."""
    
    def __init__(self) -> None:
        super().__init__('mock_wind_sensor')
        
        self.wind_pub = self.create_publisher(Vector3Stamped, '/crazyflie/wind_sensor', 10)
        self.odom_sub = self.create_subscription(Odometry, '/crazyflie/odom', self.odom_callback, 10)
        
        self.get_logger().info('Mock Wind Sensor initialized.')

    def odom_callback(self, msg: Odometry) -> None:
        # Extract current position (to be used later for complex field equations)
        x: float = msg.pose.pose.position.x
        y: float = msg.pose.pose.position.y
        z: float = msg.pose.pose.position.z
        
        # Define the static vector field (e.g., a gentle breeze in the positive X/Y direction)
        wind_u: float = 1.0  
        wind_v: float = 0.5  
        wind_w: float = 0.0  
        
        # Package and publish
        wind_msg = Vector3Stamped()
        wind_msg.header.stamp = self.get_clock().now().to_msg()
        wind_msg.header.frame_id = 'map' 
        
        wind_msg.vector.x = wind_u
        wind_msg.vector.y = wind_v
        wind_msg.vector.z = wind_w
        
        self.wind_pub.publish(wind_msg)
        
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockWindSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
