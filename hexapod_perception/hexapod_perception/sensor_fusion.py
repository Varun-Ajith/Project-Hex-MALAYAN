#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from hexapod_interfaces.msg import RobotState
import math
import tf2_ros
import tf_transformations

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Parameters
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_width', 1000)       # pixels
        self.declare_parameter('map_height', 1000)      # pixels
        self.declare_parameter('map_update_rate', 5.0)  # Hz
        self.declare_parameter('local_map_size', 10.0)  # meters

        # Initialize map
        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.map_data = np.zeros((self.height, self.width), dtype=np.int8)
        self.robot_position = [self.width//2, self.height//2]  # Start at center
        self.robot_orientation = 0.0

        # Initialize GPS reference point (first GPS reading will be reference)
        self.ref_latitude = None
        self.ref_longitude = None
        self.earth_radius = 6371000  # meters

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        
        self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10)

        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            'map',
            10)
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'robot_pose',
            10)
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10)

        # Timer for map publishing
        self.create_timer(
            1.0/self.get_parameter('map_update_rate').value,
            self.publish_map)

        self.get_logger().info('Sensor Fusion Node initialized')

    def gps_to_xy(self, lat, lon):
        """Convert GPS coordinates to local XY coordinates"""
        if self.ref_latitude is None:
            self.ref_latitude = lat
            self.ref_longitude = lon
            return 0.0, 0.0

        d_lat = math.radians(lat - self.ref_latitude)
        d_lon = math.radians(lon - self.ref_longitude)
        lat_rad = math.radians(self.ref_latitude)

        x = self.earth_radius * d_lon * math.cos(lat_rad)
        y = self.earth_radius * d_lat

        return x, y

    def update_map_from_lidar(self, scan_data, robot_x, robot_y):
        """Update occupancy grid using LiDAR data"""
        angle = scan_data.angle_min
        for range_reading in scan_data.ranges:
            if range_reading < scan_data.range_min or range_reading > scan_data.range_max:
                angle += scan_data.angle_increment
                continue

            # Convert polar to cartesian coordinates
            x = range_reading * math.cos(angle)
            y = range_reading * math.sin(angle)

            # Transform to map coordinates
            map_x = int((robot_x + x) / self.resolution)
            map_y = int((robot_y + y) / self.resolution)

            # Update map if within bounds
            if 0 <= map_x < self.width and 0 <= map_y < self.height:
                self.map_data[map_y, map_x] = 100  # Occupied

            angle += scan_data.angle_increment

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        # Update map with new LiDAR data
        self.update_map_from_lidar(msg, 
                                 self.robot_position[0] * self.resolution,
                                 self.robot_position[1] * self.resolution)

        # Publish transform
        self.publish_transform()

    def gps_callback(self, msg):
        """Process incoming GPS data"""
        # Convert GPS to local coordinates
        x, y = self.gps_to_xy(msg.latitude, msg.longitude)

        # Update robot position in map coordinates
        self.robot_position[0] = int(x / self.resolution) + self.width//2
        self.robot_position[1] = int(y / self.resolution) + self.height//2

        # Create and publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0

        # Set orientation (assuming robot's heading is known)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.robot_orientation)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose_msg)

    def publish_transform(self):
        """Publish transform from map to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = self.robot_position[0] * self.resolution - (self.width * self.resolution)/2
        t.transform.translation.y = self.robot_position[1] * self.resolution - (self.height * self.resolution)/2
        t.transform.translation.z = 0.0

        # Set rotation
        q = tf_transformations.quaternion_from_euler(0, 0, self.robot_orientation)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_map(self):
        """Publish the occupancy grid map"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Set origin to center of map
        msg.info.origin.position.x = -(self.width * self.resolution)/2
        msg.info.origin.position.y = -(self.height * self.resolution)/2
        msg.info.origin.position.z = 0.0
        
        # Convert numpy array to 1D list
        msg.data = self.map_data.flatten().tolist()

        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()