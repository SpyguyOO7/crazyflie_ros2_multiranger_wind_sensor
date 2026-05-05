#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import math
import time
import numpy as np
import scipy.ndimage
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class EntropyMapper(Node):
    def __init__(self):
        super().__init__('entropy_mapper')
        
        # --- Parameters ---
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        clean_prefix = robot_prefix.strip('/')

        # --- Map Configuration ---
        # 0.2m (20cm) resolution is a good balance. 
        # Keeps physical walls crisp, but caps markers at 10,000 so RViz won't crash
        self.map_res = 0.2  
        self.map_size_x = 4.0  
        self.map_size_y = 4.0  
        self.map_width = int(self.map_size_x / self.map_res)
        self.map_height = int(self.map_size_y / self.map_res)
        
        # Physical Map (1D array for OccupancyGrid, -1 = unknown, 0 = free, 100 = wall)
        self.map_data = np.full(self.map_width * self.map_height, -1, dtype=np.int8)
        
        # Entropy Map (2D NumPy float array, 1.0 = absolute uncertainty)
        self.entropy_map = np.ones((self.map_height, self.map_width), dtype=np.float32)
        
        # Algorithm Tuning
        self.lambda_decay = 0.5  
        self.explore_radius = 5  

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, f'{robot_prefix}/cmd_vel', 10)
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/entropy_markers', 10)
        
        # --- Subscribers ---
        self.odom_sub = self.create_subscription(Odometry, f'{robot_prefix}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, f'{robot_prefix}/scan', self.scan_callback, 10)
        self.wind_sub = self.create_subscription(Vector3Stamped, '/wind_sensor', self.wind_callback, 10)
        
        self.latest_wind = None
        
        # --- TF Broadcaster ---
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{clean_prefix}/odom' 
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # --- State Variables ---
        self.drone_x, self.drone_y, self.drone_z, self.drone_yaw = None, None, None, None
        self.target_x, self.target_y, self.target_z, self.target_yaw = 0.0, 0.0, 0.25, 0.0
        
        self.state = 'INIT'
        self.measure_start_time = 0.0
        
        # P-Controller Gains
        self.Kp_xy, self.Kp_z, self.Kp_yaw = 0.8, 1.0, 1.0
        self.max_v_xy, self.max_v_z, self.max_w_yaw = 0.5, 0.3, 0.5
        
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Dual Mapper (Physical + Entropy) Initialized.")

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------
    def odom_callback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.drone_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def wind_callback(self, msg):
        self.latest_wind = msg.vector

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_grid_coords(self, x, y):
        gx = int((x + self.map_size_x / 2.0) / self.map_res)
        gy = int((y + self.map_size_y / 2.0) / self.map_res)
        return gx, gy

    # ---------------------------------------------------------
    # PHYSICAL MAP (LIDAR)
    # ---------------------------------------------------------
    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))
        return points

    def scan_callback(self, msg):
        if self.drone_x is None or self.drone_yaw is None: return

        drone_gx, drone_gy = self.get_grid_coords(self.drone_x, self.drone_y)
        map_updated = False

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + (i * msg.angle_increment) + self.drone_yaw
            hit_x = self.drone_x + r * math.cos(angle)
            hit_y = self.drone_y + r * math.sin(angle)
            hit_gx, hit_gy = self.get_grid_coords(hit_x, hit_y)

            points = self.bresenham(drone_gx, drone_gy, hit_gx, hit_gy)
            for p in points[:-1]:
                if 0 <= p[0] < self.map_width and 0 <= p[1] < self.map_height:
                    idx = p[1] * self.map_width + p[0]
                    self.map_data[idx] = 0 # Free space
            
            if 0 <= hit_gx < self.map_width and 0 <= hit_gy < self.map_height:
                idx = hit_gy * self.map_width + hit_gx
                self.map_data[idx] = 100 # Wall
                
            map_updated = True

        if map_updated:
            self.publish_physical_map()

    def publish_physical_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map' 
        
        grid_msg.info.resolution = self.map_res
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = -self.map_size_x / 2.0
        grid_msg.info.origin.position.y = -self.map_size_y / 2.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = self.map_data.tolist()
        self.map_pub.publish(grid_msg)

    # ---------------------------------------------------------
    # ENTROPY MATH & PATH PLANNING
    # ---------------------------------------------------------
    def update_entropy_map(self):
        if self.latest_wind is None: return
        
        wind_u, wind_v = self.latest_wind.x, self.latest_wind.y
        measure_gx, measure_gy = self.get_grid_coords(self.drone_x, self.drone_y)
        
        if not (0 <= measure_gx < self.map_width and 0 <= measure_gy < self.map_height): return

        y_indices, x_indices = np.indices((self.map_height, self.map_width))
        dx = x_indices - measure_gx
        dy = y_indices - measure_gy
        distances = np.sqrt(dx**2 + dy**2) * self.map_res
        distances[measure_gy, measure_gx] = 1e-6 
        
        decay = np.exp(-distances / self.lambda_decay)
        
        wind_mag = math.sqrt(wind_u**2 + wind_v**2) + 1e-6
        dot_product = (dx * self.map_res * wind_u) + (dy * self.map_res * wind_v)
        similarity = np.abs(dot_product / (distances * wind_mag))
        
        similarity[measure_gy, measure_gx] = 1.0
        
        reduction_factor = 1.0 - (similarity * decay)
        self.entropy_map *= reduction_factor
        self.entropy_map[measure_gy, measure_gx] = 0.0 
        
        self.publish_entropy_markers()

    def get_next_waypoint(self):
        ig_map = scipy.ndimage.uniform_filter(self.entropy_map, size=self.explore_radius) * (self.explore_radius**2)
        
        y_indices, x_indices = np.indices((self.map_height, self.map_width))
        cell_x = (x_indices * self.map_res) - (self.map_size_x / 2.0)
        cell_y = (y_indices * self.map_res) - (self.map_size_y / 2.0)
        distances = np.sqrt((cell_x - self.drone_x)**2 + (cell_y - self.drone_y)**2)
        distances = np.maximum(distances, 0.5) 
        
        utility_map = ig_map / distances
        
        # --- APPLY MASKS ---
        # 1. Mask out areas we already know well
        utility_map[self.entropy_map < 0.1] = -1.0
        
        # 2. Mask out Physical Walls 
        # Reshape the 1D physical map into a 2D numpy array to match the utility map perfectly
        physical_2d = self.map_data.reshape((self.map_height, self.map_width))
        utility_map[physical_2d >= 50] = -1.0 
        
        # --- FIND BEST TARGET ---
        if np.max(utility_map) <= 0.0:
            self.get_logger().info("Map complete or no valid targets left! Hovering.")
            return self.drone_x, self.drone_y
            
        max_idx = np.argmax(utility_map)
        max_gy, max_gx = np.unravel_index(max_idx, utility_map.shape)
        
        target_x = (max_gx * self.map_res) - (self.map_size_x / 2.0)
        target_y = (max_gy * self.map_res) - (self.map_size_y / 2.0)
        
        return target_x, target_y

    def publish_entropy_markers(self):
        marker_array = MarkerArray()
        y_indices, x_indices = np.nonzero(self.entropy_map > -1)
        
        for gy, gx in zip(y_indices, x_indices):
            entropy = self.entropy_map[gy, gx]
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "entropy"
            marker.id = int(gy * self.map_width + gx)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = (gx * self.map_res) - (self.map_size_x / 2.0) + (self.map_res / 2.0)
            marker.pose.position.y = (gy * self.map_res) - (self.map_size_y / 2.0) + (self.map_res / 2.0)
            marker.pose.position.z = 0.1 
            
            marker.scale.x = self.map_res * 0.95
            marker.scale.y = self.map_res * 0.95
            marker.scale.z = 0.01
            
            marker.color.r = float(entropy)
            marker.color.g = float(1.0 - entropy)
            marker.color.b = 0.0
            marker.color.a = 0.4 
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

    # ---------------------------------------------------------
    # CONTROL LOOP
    # ---------------------------------------------------------
    def control_loop(self):
        if self.drone_x is None: return

        if self.state == 'INIT':
            self.target_x, self.target_y, self.target_z = self.drone_x, self.drone_y, 0.5
            self.target_yaw = self.drone_yaw
            self.state = 'TAKEOFF'
            self.get_logger().info("Taking off...")

        elif self.state == 'TAKEOFF':
            if abs(self.target_z - self.drone_z) < 0.05:
                self.publish_entropy_markers()
                self.state = 'PLAN_PATH'

        elif self.state == 'PLAN_PATH':
            self.target_x, self.target_y = self.get_next_waypoint()
            self.target_yaw = math.atan2(self.target_y - self.drone_y, self.target_x - self.drone_x)
            self.state = 'FLYING'
            self.get_logger().info(f"Navigating to Target: X={self.target_x:.2f}, Y={self.target_y:.2f}")

        elif self.state == 'FLYING':
            dist_to_target = math.sqrt((self.target_x - self.drone_x)**2 + (self.target_y - self.drone_y)**2)
            if dist_to_target < 0.15: 
                self.state = 'MEASURE'
                self.measure_start_time = time.time()
                self.get_logger().info("Target reached. Hovering and measuring...")
                
        elif self.state == 'MEASURE':
            if time.time() - self.measure_start_time > 1.5:
                self.update_entropy_map()
                self.state = 'PLAN_PATH' 

        # P-Controller Execution
        error_x = self.target_x - self.drone_x
        error_y = self.target_y - self.drone_y
        error_z = self.target_z - self.drone_z
        error_yaw = self.normalize_angle(self.target_yaw - self.drone_yaw)

        error_body_x = error_x * math.cos(self.drone_yaw) + error_y * math.sin(self.drone_yaw)
        error_body_y = -error_x * math.sin(self.drone_yaw) + error_y * math.cos(self.drone_yaw)

        cmd_vx = np.clip(self.Kp_xy * error_body_x, -self.max_v_xy, self.max_v_xy)
        cmd_vy = np.clip(self.Kp_xy * error_body_y, -self.max_v_xy, self.max_v_xy)
        cmd_vz = np.clip(self.Kp_z * error_z, -self.max_v_z, self.max_v_z)
        cmd_wyaw = np.clip(self.Kp_yaw * error_yaw, -self.max_w_yaw, self.max_w_yaw)

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(cmd_vx), float(cmd_vy), float(cmd_vz)
        msg.angular.z = float(cmd_wyaw)
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EntropyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_vel_pub.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
