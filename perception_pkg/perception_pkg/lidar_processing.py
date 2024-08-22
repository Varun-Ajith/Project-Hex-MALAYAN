#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from my_robot_interfaces.action import NavigateHuman
import numpy as np

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__("lidar_processing")
        self.scan_subscription_ = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10)
        self.human_detection_subscription = self.create_subscription(
            Bool,
            "human_detection",
            self.human_detection_callback,
            10)
        self.detection_service = self.create_service(
            SetBool,
            "toggle_human_detection",
            self.toggle_human_detection_callback)
        self.action_server_ = ActionServer(
            self,
            NavigateHuman,
            "navigate_to_human",
            self.execute_navigation_callback)
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            "movement_commands",
            10)
        self.detection_active = False
        self.latest_scan_data = None
        self.human_detected = False
        self.get_logger().info("Lidar Processing node initialized...")
    
    def scan_callback(self, msg):
        if not self.detection_active :
            return
        self.latest_scan_data = np.array(msg.ranges)
        self.get_logger().info("Recieved scan data..")

        if self.human_detected:
            self.avoid_obstacle()

    def human_detection_callback(self, msg):
        if not self.detection_active:
            return
        self.human_detected = msg.data
        if self.human_detected:
            self.get_logger().info("Human detected! Initiating navigation...")
            self.navigate_to_human()

    def toggle_human_detection_callback(self, request, response):
        self.detection_active = request.data
        response.success = True
        response.message = "Human detection "+(" activated " if self.detection_active else "deactivated")
        self.get_logger().info(response.message)
        return response

    def execute_navigation_callback(self, goal_handle):
        self.get_logger().info("Executing navigation to human...")
        if self.human_detected:
            self.navigate_to_human()
        goal_handle.succeed()
        result = NavigateHuman.Result()
        result.success = True
        return result
    
    def avoid_obstacle(self):
        if self.latest_scan_data is not None:
            min_distance = np.min(self.latest_scan_data)
            if min_distance < 0.5:
                self.get_logger().warn("Obstacle detected! Stoppinf or rerouting...")
                safe_direction =np.argmax(self.latest_scan_data)
                safe_distance = self.latest_scan_data[safe_direction]

                if safe_distance < 0.5:
                    self.get_logger().info(f"Turning toward safe direction {safe_direction} degrees ")
                    self.turn_robots(safe_direction)
                else:
                    self.get_logger().info("There is no safe route . Stopping the robot...")
                    self.stop_robot()
    
    def turn_robot(self, direction):
        if direction < len(self.latest_scan_data)//2:
            self.publish_turn_command(left = True)
        else:
            self.publish_turn_command(left = False)
    
    def stop_robot(self):
        stop_command =Float64MultiArray()
        stop_command.data = [0.0]*len(self.latest_scan_data)
        self.publisher_.publish(stop_command)
        self.get_logger().info("Robot stopped!")
    
    def publish_turn_command(self, left):
        turn_command = Float64MultiArray()
        if left:
            turn_command.data = [-0.51, 0.52]
        else:
            turn_command.data = [0.51, -0.52]
        self.publisher_.publish(turn_command)
        self.get_logger().info("Published turn command!")
    
    def navigate_to_human(self):
        self.get_logger().info("Navigating to human...")
        if self.latest_scan_data is not None:
            direction = np.argmin(self.latest_scan_data)
            if self.latest_scan_data[direction] < 0.5:
                self.get_logger().warn("Path blocked!! finding an alternate way...")
                safe_direction = np.argmax(self.latest_scan_data)
                self.turn_robot(safe_direction)
            else:
                self.get_logger().info(f"Moving towards human in {direction} degrees ")       
    
def main(args = None):
    rclpy.init(args =args)
    node =LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()