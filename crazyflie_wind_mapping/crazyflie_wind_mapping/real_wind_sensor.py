#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import tf_transformations
from collections import deque
try:
    import liveWindUKF
    HAS_UKF = True
except ImportError:
    print('failed to import liveWindUKF, using unfiltered winds')
    HAS_UKF = False


def average_tuples(tuples: list[tuple[float, ...]]) -> tuple[float, ...]:
    """
    Calculates the element-wise average of a list of tuples.
    Assumes all tuples in the list are of the same length.
    """
    if not tuples:
        return ()

    num_tuples = len(tuples)
    
    # zip(*tuples) groups the i-th elements of each tuple together
    return tuple(sum(elements) / num_tuples for elements in zip(*tuples))
class RealWindSensor(Node):
    """
    Subscribes to raw drone-relative wind and body-frame odometry to calculate
    and publish the room-relative wind vector in the global frame.
    """

    def __init__(self) -> None:
        super().__init__('real_wind_sensor')

        self.wind_zero_samps = [(None,None,None)]*100
        self.wind_zero = (None,None,None)
        self.num_zero_samps = 0;
        
        self._latest_odom: Optional[Odometry] = None
        
        self._wind_world_history: deque[np.ndarray] = deque(maxlen=10)
        
        self._odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10
        )

        self._wind_raw_sub = self.create_subscription(
            Vector3Stamped,
            'wind',
            self._wind_raw_callback,
            10
        )

        self._wind_world_pub = self.create_publisher(
            Vector3Stamped,
            'wind_sensor',
            10
        )

        if not HAS_UKF:
            self.get_logger().warning('liveWindUKF module not found. Falling back to purely kinematic calculation.')

        self.get_logger().info('Real Wind Sensor node initialized.')

    def _odom_callback(self, msg: Odometry) -> None:
        """Stores the latest odometry message for use in the wind calculation."""
        self._latest_odom = msg

    def _wind_raw_callback(self, msg: Vector3Stamped) -> None:
        """
        Computes the global wind vector by fusing the body-frame flow sensor data
        with the body-frame drone velocity, then rotating the result into the world frame.
        """
        if self._latest_odom is None:
            return

        # Extract drone velocity from the child frame (body frame) Twist
        vx_body: float = self._latest_odom.twist.twist.linear.x
        vy_body: float = self._latest_odom.twist.twist.linear.y
        vz_body: float = self._latest_odom.twist.twist.linear.z
        
        # Extract raw flow sensor data (body frame)
        bx: float = msg.vector.x
        by: float = msg.vector.y
        bz: float = msg.vector.z

        if self.num_zero_samps < 100:
            self.wind_zero_samps[self.num_zero_samps] = (bx, by,bz)
            self.num_zero_samps = self.num_zero_samps + 1
            return
        if self.num_zero_samps == 100:
            self.wind_zero = average_tuples(self.wind_zero_samps)
        # Both vectors are in the body frame, so they can be summed directly
        wind_body = np.array([bx-self.wind_zero[0], by-self.wind_zero[1], bz-self.wind_zero[2]])

        # Extract quaternion
        q = self._latest_odom.pose.pose.orientation

        # 1. Calculate the magnitude of the quaternion
        q_norm: float = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        print(f"q_norm:{q_norm}")
        # 2. Normalize the quaternion to ensure a pure rotation matrix (preventing scaling)
        if q_norm > 0.0:
            qx, qy, qz, qw = q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm
        else:
             # Fallback to an identity quaternion if something goes critically wrong
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        # Build the rotation matrix using the normalized quaternion
        rotation_matrix = tf_transformations.quaternion_matrix([qx, qy, qz, qw])[:3, :3]

        # Rotate the combined body-frame wind vector into the fixed world frame
        kinematic_wind_world = wind_body
        #kinematic_wind_world = wind_bodyv

        self._wind_world_history.append(kinematic_wind_world)
        smoothed_wind_world = np.mean(self._wind_world_history, axis=0)
        
        final_wind_x: float = float(smoothed_wind_world[0])
        final_wind_y: float = float(smoothed_wind_world[1])
        final_wind_z: float = float(smoothed_wind_world[2])


        # if HAS_UKF:
        #     try:
        #         # The UKF requires the raw magnitudes. Velocity magnitudes are invariant to coordinate frames.
        #         flow_mag = math.sqrt(bx**2 + by**2)
        #         drone_v = math.sqrt(vx_body**2 + vy_body**2)
                
        #         airspeed, wind_mag, empirical = liveWindUKF.run_ukf(2 * flow_mag, drone_v)
                
        #         # Apply the filtered magnitude to the rotated world-frame flow angle
        #         world_flow_angle = math.atan2(kinematic_wind_world[1], kinematic_wind_world[0])
                
        #         final_wind_x = wind_mag * math.cos(world_flow_angle)
        #         final_wind_y = wind_mag * math.sin(world_flow_angle)
        #         final_wind_z = 0.0  # Assuming 2D planar UKF
        #     except Exception as e:
        #         self.get_logger().error(f'UKF calculation failed: {e}')
        #         return

        world_wind_msg = Vector3Stamped()
        world_wind_msg.header.stamp = self.get_clock().now().to_msg()
        world_wind_msg.header.frame_id = self._latest_odom.header.frame_id
        
        world_wind_msg.vector.x = final_wind_x
        world_wind_msg.vector.y = final_wind_y
        world_wind_msg.vector.z = final_wind_z
        
        self._wind_world_pub.publish(world_wind_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealWindSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

