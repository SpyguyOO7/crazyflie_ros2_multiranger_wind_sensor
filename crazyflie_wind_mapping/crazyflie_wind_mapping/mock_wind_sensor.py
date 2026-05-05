#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

class MockWindSensor(Node):
    """Simulates a hardware wind sensor by generating a synthetic 2D fluid dipole field."""
    
    def __init__(self) -> None:
        super().__init__('mock_wind_sensor')
        
        self.wind_pub = self.create_publisher(Vector3Stamped, 'wind_sensor', 10)
        self.odom_sub = self.create_subscription(Odometry, '/crazyflie/odom', self.odom_callback, 10)
        
        # --- Dipole Field Parameters ---
        # High pressure source (e.g., an AC supply vent)
        self.source_x = 1.0
        self.source_y = 0.0
        
        # Low pressure sink (e.g., an AC return vent)
        self.sink_x = -1.0
        self.sink_y = 0.0
        
        # Strength of the flow
        self.flow_strength = 8.0 
        
        # Prevent division by zero if drone is directly on top of a pole
        self.epsilon = 0.1 
        
        self.get_logger().info('Mock Wind Sensor initialized with Dipole Field.')

    def odom_callback(self, msg: Odometry) -> None:
        # Extract current drone position
        x: float = msg.pose.pose.position.x
        y: float = msg.pose.pose.position.y
        
        # Vectors from poles to drone
        dx_source = x - self.source_x
        dy_source = y - self.source_y
        
        dx_sink = x - self.sink_x
        dy_sink = y - self.sink_y
        
        # Squared distances (with epsilon to avoid singularities)
        dist_sq_source = (dx_source**2 + dy_source**2) + self.epsilon
        dist_sq_sink = (dx_sink**2 + dy_sink**2) + self.epsilon
        
        # 2D Fluid Dipole Equation: V = Q * [ (d_source / r_source^2) - (d_sink / r_sink^2) ]
        wind_u: float = self.flow_strength * ((dx_source / dist_sq_source) - (dx_sink / dist_sq_sink))
        wind_v: float = self.flow_strength * ((dy_source / dist_sq_source) - (dy_sink / dist_sq_sink))
        wind_w: float = 0.0  # Keep it 2D for the occupancy grid mapping
        
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
