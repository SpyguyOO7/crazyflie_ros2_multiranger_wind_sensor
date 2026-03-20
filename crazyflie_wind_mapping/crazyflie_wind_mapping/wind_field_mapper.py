#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from typing import Dict, Tuple

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class WindFieldMapper(Node):
    """Records incoming wind vectors into a discrete spatial grid and publishes them for RViz."""
    
    def __init__(self) -> None:
        super().__init__('wind_field_mapper')
        
        # Configuration
        self.grid_resolution: float = 0.5  # Size of each voxel in meters
        self.vector_scale: float = 0.3     # Visual scaling for RViz arrows
        
        # State
        # Maps a discrete 3D grid coordinate (x, y, z) to a Wind Vector
        self.wind_map: Dict[Tuple[int, int, int], Vector3Stamped] = {}
        self.current_position: Point | None = None
        
        # Topics
        self.odom_sub = self.create_subscription(Odometry, '/crazyflie/odom', self.odom_callback, 10)
        self.wind_sub = self.create_subscription(Vector3Stamped, '/crazyflie/wind_sensor', self.wind_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/crazyflie/wind_field_markers', 10)
        
        # Publish the map to RViz every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Wind Field Mapper initialized.')

    def odom_callback(self, msg: Odometry) -> None:
        self.current_position = msg.pose.pose.position

    def wind_callback(self, msg: Vector3Stamped) -> None:
        if self.current_position is None:
            return  # Drop measurement if we don't know our location
            
        # Discretize continuous position into a grid cell index
        grid_x = math.floor(self.current_position.x / self.grid_resolution)
        grid_y = math.floor(self.current_position.y / self.grid_resolution)
        grid_z = math.floor(self.current_position.z / self.grid_resolution)
        
        grid_cell = (grid_x, grid_y, grid_z)
        
        # Save or update the wind vector for this specific cell
        self.wind_map[grid_cell] = msg

    def publish_markers(self) -> None:
        if not self.wind_map:
            return
            
        marker_array = MarkerArray()
        
        for idx, (cell, wind_msg) in enumerate(self.wind_map.items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'wind_vectors'
            marker.id = idx
            
            # ARROW markers require exactly two points: [Start, End]
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Start point is the center of the voxel
            start_point = Point()
            start_point.x = cell[0] * self.grid_resolution + (self.grid_resolution / 2.0)
            start_point.y = cell[1] * self.grid_resolution + (self.grid_resolution / 2.0)
            start_point.z = cell[2] * self.grid_resolution + (self.grid_resolution / 2.0)
            
            # End point is calculated using the wind vector and visual scale
            end_point = Point()
            end_point.x = start_point.x + (wind_msg.vector.x * self.vector_scale)
            end_point.y = start_point.y + (wind_msg.vector.y * self.vector_scale)
            end_point.z = start_point.z + (wind_msg.vector.z * self.vector_scale)
            
            marker.points = [start_point, end_point]
            
            # Arrow formatting: scale.x is shaft diameter, y is head diameter, z is head length
            marker.scale.x = 0.05 
            marker.scale.y = 0.1  
            marker.scale.z = 0.1  
            
            # Light blue color
            marker.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=0.8) 
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WindFieldMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
