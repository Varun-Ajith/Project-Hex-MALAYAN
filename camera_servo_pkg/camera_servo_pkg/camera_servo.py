#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import termios
import tty
import sys

class CameraServoNode(Node):
    def __init__(self):
        super().__init__("camera_servo")
        self.servo_publisher_ = self.create_publisher(
            Float64,
            "camera_servo_angle",
            10)
        
        self.servo_angle = 90.0
        self.publish_servo_angle()
        self.get_logger().info("Camera servo initialized. Press 'L' to move left, 'R' to move right, 'Q' to quit.")
        self.listen_for_input()

    def listen_for_input(self):
        old_setting = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin)
            while True:
                key = sys.stdin.read(1)
                if key in ('L', 'l'):
                    self.move_servo_left()
                elif key in ('R', 'r'):
                    self.move_servo_right()
                elif key in ('Q', 'q'):
                    self.get_logger().info("Quitting...")
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    
    def move_servo_left(self):
        if self.servo_angle > 5:
            self.servo_angle -= 5.0
            self.get_logger().info(f"Servo moved left to {self.servo_angle} degrees")
            self.publish_servo_angle()
        else:
            self.get_logger().info("Servo already at minimum angle")
    
    def move_servo_right(self):
        if self.servo_angle < 175.0:
            self.servo_angle += 5.0
            self.get_logger().info(f"Servo moved right to {self.servo_angle} degrees")
            self.publish_servo_angle()
        else:
            self.get_logger().info("Servo already at maximum angle")
    
    def publish_servo_angle(self):
        msg = Float64()
        msg.data = self.servo_angle
        self.servo_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
