#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import Servo
from std_msgs.msg import Float64MultiArray
from time import sleep

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__("servo_controller")
        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            "joint_commands",
            self.listener_callback,
            10)
        self.subscription_
        self.servos_ = [Servo(pin) for pin in range (2,20)]
        self.get_logger().info("Servo interface initialized...")
    
    def listener_callback(self,msg):
        joint_positions = msg.data
        if len(joint_positions) != len(self.servos_):
            self.get_logger().info("Mismatch between joint_positions length and number of servos")
            return
        self.get_logger().info(f"Recieved joint positions: {joint_positions}")
        self.move_servos(joint_positions)
    
    def move_servos(self,joint_positions):
        for i, position in enumerate (joint_positions):
            position = max(min(position,1.0), -1.0)
            self.servos_[i].value = position
            self.get_logger().info(f"moving servo {i} to position {position}")
            sleep(0.1)


def main(args = None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()