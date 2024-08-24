#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import random
import math
import numpy as np

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning")

        self.declare_parameter('map_resolution', 1.0)
        self.declare_parameter('max_iteration', 1000)
        self.declare_parameter('step_size', 1.0)

        self.map_resolution = self.get_parameter('map_resolution').value
        self.max_iteration = self.get_parameter('max_iteration').value
        self.step_size = self.get_parameter('step_size').value

        self.path_publisher_ = self.create_publisher(
            PoseArray,
            "path",
            10)
        self.map_subscriber_ = self.create_subscription(
            OccupancyGrid,
            "map",
            self.map_callback,
            10)
        self.start_goal_subscriber = self.create_subscription(
            PoseArray,
            "start_goal",
            self.start_goal_callback,
            10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_callback,
            10)
        
        self.grid = None
        self.start = None
        self.goal = None
        self.latest_scan_data = None
        self.get_logger().info("Path planning initialized...")

    def map_callback(self, msg):
        self.grid = msg
        self.obstacles = []
        width = self.grid.info.width
        height = self.grid.info.height
        resolution = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if self.grid.data[index] > 50:
                    obstacle_x = origin_x + x * resolution
                    obstacle_y = origin_y + y * resolution
                    self.obstacles.append((obstacle_x, obstacle_y))

    def start_goal_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn("Start and Goal poses are not provided")
            return

        self.start = msg.poses[0]
        self.goal = msg.poses[1]
        self.get_logger().info(f"Start pose: {self.start}, Goal pose: {self.goal}")

        self.plan_path()

    def lidar_callback(self, msg):
        self.latest_scan_data = msg.ranges
        self.get_logger().info("Received LiDAR data")

    def plan_path(self):
        if self.start is None or self.goal is None:
            self.get_logger().warn("Start and goal poses are not set")
            return

        path = self.rrt_star(self.start, self.goal)
        path_msg = PoseArray()
        path_msg.poses = path
        self.path_publisher_.publish(path_msg)
        self.get_logger().info("Path published")

    def rrt_star(self, start, goal):
        path = []
        #still in work
        return path

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
