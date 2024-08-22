#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from my_robot_interfaces.action import NavigateHuman
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy

class MasterNode(Node):
    def __init__(self):
        super().__init__("master_node")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,
            "cmd_vel",
            10)
        self.camera_controller_publisher_ = self.create_publisher(
            Bool,
            "camera_control",
            10)
        self.human_detected_subscriber_ = self.create_subscription(
            Bool,
            "human_detection",
            self.human_detected_callback,
            10)
        self.joy_subscriber_ = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            10)
        
        self.navigate_human_client = ActionClient(
            self,
            NavigateHuman,
            "navigate_to_human")
        
        self.human_detection_client = self.create_client(
            SetBool,
            "toggle_human_detection")

        self.get_logger().info("Master node has been initialized...")

    def human_detected_callback(self, msg):
        if msg.data:
            self.get_logger().info("Human detected! Starting navigation...")
            self.start_navigation_to_human()
        else:
            self.get_logger().warn("No human found")

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1] * 0.50
        twist.angular.z = msg.axes[0] * 1.0
        self.cmd_vel_publisher_.publish(twist)

        if msg.buttons[0] == 1:
            self.camera_controller_publisher_.publish(Bool(data=True))
        elif msg.buttons[1] == 1:
            self.camera_controller_publisher_.publish(Bool(data=False))

    def start_navigation_to_human(self):
        if not self.navigate_human_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available. Cannot start navigation.")
            return

        goal_msg = NavigateHuman.Goal()
        self.navigate_human_client.send_goal_async(goal_msg, feedback_callback=self.navigation_feedback_callback)

    def navigation_feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Navigating to human: {feedback_msg.feedback}")

    def toggle_human_detection(self, active: bool):
        if not self.human_detection_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available. Cannot toggle human detection.")
            return

        req = SetBool.Request()
        req.data = active
        self.human_detection_client.call_async(req)
        self.get_logger().info(f"Human detection {'activated' if active else 'deactivated'}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    node.destroy_node()  # Fixed missing parentheses
    rclpy.shutdown()

if __name__ == "__main__":
    main()
