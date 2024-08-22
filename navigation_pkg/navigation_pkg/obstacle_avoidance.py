#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.lidar_subscriber_ = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_callback,
            10)
        self.movement_publisher_ = self.create_publisher(
            Float64MultiArray,
            "movement_commands",
            10)
        
        self.declare_parameter('safe_distance', 1.0)
        self.safe_distance_ = self.get_parameter('safe_distance').value

        self.latest_scan_data = None
        self.get_logger().info("Obstacle avoidance node initialized...")
    
    def lidar_callback(self, msg):
        self.latest_scan_data = msg.ranges
        self.process_lidar_data()
    
    def process_lidar_data(self):
        if self.latest_scan_data is None:
            return
        min_distance = min(self.latest_scan_data)
        min_index = self.latest_scan_data.index(min_distance)
        if min_distance < self.safe_distance_:
            self.get_logger().info(f"Obstacle detected at {min_distance:.2f} meters")
            self.avoid_obstacle(min_index, min_distance)
    
    def avoid_obstacle(self, direction_index, distance):
        left_turn = direction_index < len(self.latest_scan_data) // 2
        turn_command = Float64MultiArray()
        if left_turn:
            turn_command.data = [-0.51, 0.52]
            self.get_logger().info("Turning left to avoid obstacle..")
        else:
            turn_command.data = [0.51, -0.52]
            self.get_logger().info("Turning right to avoid obstacle...")
        
        self.movement_publisher_.publish(turn_command)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
