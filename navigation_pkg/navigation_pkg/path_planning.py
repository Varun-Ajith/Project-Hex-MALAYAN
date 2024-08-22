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

        self.declare_parameter('map_resolution',1.0)
        self.declare_parameter("max_iteration", 1000)
        self.declare_parameter("step_size", 1.0)

        self.map_resolution = self.get_parameter("map_resolution").value
        self.max_iteration = self.get_parameter("max_iteration").value
        self.step_size = self.get_parameter("step_size").value

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

        for y in range(width):
            for x in range(height):
                index = y * width + x
                if self.grid.data[index] > 50:
                    obstacle_x = origin_x + x* resolution
                    obstacle_y = origin_y + y *resolution
                    self.obstacles.append((obstacle_x, obstacle_y))
        
        self.get_logger().info("Map updated with obstacle")

    def start_goal_callback(self, msg):
        poses = msg.poses
        if len(poses) >= 2:
            self.start = (poses[0].position.x, poses[0].position.y) 
            self.goal = (poses[1].position.x, poses[1].position.y)
            self.plan_path()
        
    def lidar_callback(self, msg):
        self.latest_scan_data = np.array(msg.ranges)
    
    def plan_path(self):
        if self.start or not self.goal or not self.grid:
            self.get_logger().warn("Star, goal or map not set!")
            return
        path = self.rrt_algorithm(self.start, self.goal)
        self.publish_path(path)

    def rrt_algorithm(self, start, goal):
        nodes = [start]
        edges = []
        for _ in range(self.max_iteration):
            random_point = (random.uniform(0, self.grid.info.width * self.map_resolution),
                            random.uniform(0, self.grid.info.height * self.map_resolution))
            nearest_node = min(nodes, key = lambda n: np.linalg.norm(np.array(n) - np.array(random_point)))
            direction = np.array(random_point) - np.array(nearest_node)
            distance = np.linalg(direction)
            direction /= distance

            new_node = (nearest_node[0]+direction[0] * self.step_size,
                        nearest_node[1] + direction[1] * self.step_size)
            
            if self.is_free(new_node) and self.is_free_from_obstacle(new_node):
                nodes.append(new_node)
                edges.append((nearest_node, new_node))

                if np.linalg.norm(np.array(new_node) - np.array(goal)) < self.step_size:
                    nodes.append(goal)
                    edges.append((new_node, goal))
                    break
        path = self.extract_path(nodes, edges, goal)
        return path
    
    def is_free(self, point):
        grid_x = int(point[0] / self.map_resolution)
        grid_y = int(point[1] / self.map_resolution)

        if grid_x < 0 or grid_x > self.grid.info.width or grid_y < 0 or grid_y > self.grid.info.height:
            return False
        return self.grid.data[grid_y * self.grid.info.width + grid_x] == 0
    
    def is_free_from_obstacle(self, point):
        if self.latest_scan_data is not None:
            point_angle = int(math.degrees(math.atan2(point[1] - self.start[1], point[0] - self.start[0])) % 360)
            if point_angle < len(self.latest_scan_data):
                distance_to_point = np.linalg.norm(np.array(point) - np.array(self.start))
                if self.latest_scan_data[point_angle] < distance_to_point:
                    return False
        return True
    
    def extract_path(self, nodes, edges, goal):
        path = []
        current_node =goal
        path.append(current_node)
        
        while current_node != nodes[0]:
            for edge in edges:
                if edge[1] == current_node:
                    current_node = edge[0]
                    path.append(current_node)
                    break
        path.reverse()
        return path

    def publish_path(self, path):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for (x,y) in path:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose_array.poses.append(pose)
        self.path_publisher_.publish(pose_array)

    
def main(args = None):
    rclpy.init(args = args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
