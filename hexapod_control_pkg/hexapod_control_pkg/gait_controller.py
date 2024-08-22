#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from my_robot_interfaces.srv import ResetPosition
from my_robot_interfaces.msg import LegCommand
from my_robot_interfaces.action import MoveSequence

class GaitControllerNode(Node):
    def __init__(self):
        super().__init__("gait_controller")
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            "joint_commands",
            10)
        self.srv_ = self.create_service(
            Empty,
            "reset_position",
            self.reset_position_callback)
        self.action_server_ = ActionServer(
            self,
            MoveSequence,
            "execute_sequence",
            self.execute_sequence_callback)
        self.get_logger().info("Gait controller initialized.....")

    def reset_position_callback(self, request, response):
        self.publish_joint_positions(self.get_default_joint_positions())
        self.get_logger().info("Robot position reset")
        response.success = True
        return response

    def execute_sequence_callback(self, goal_handle):
        self.get_logger().info("Executing movement sequence...")
        feedback_msg = MoveSequence.Feedback()
        for step in range(3):
            joint_positions = self.calculate_tripod_gait_step(step)
            self.publish_joint_positions(joint_positions)
            feedback_msg.current_step = step
            self.action_server_.publish_feedback(feedback_msg)
            self.get_logger().info(f"step {step+1}: Joint position updated")
            rclpy.spin_once(self, timeout_sec=1)
        goal_handle.succeed()
        result = MoveSequence.Result()
        result.success = True
        return result
    
    def calculate_tripod_gait_step(self, step):
        joint_positions = []
        if step == 0:
            joint_positions = [
                20.0, 60.0, 30.0,
                00.0, 30.0, 60.0,
                00.0, 30.0, 60.0,
                -20.0, 60.0, 30.0,
                20.0, 60.0, 30.0,
                00.0, 30.0, 60.0
            ]
        elif step == 1:
            joint_positions = [
                00.0, 30.0, 60.0,
                20.0, 60.0, 30.0,
                20.0, 60.0, 30.0,
                00.0, 30.0, 60.0,
                00.0, 30.0, 60.0,
                -20.0, 60.0, 30.0
            ]
        else:
            joint_positions = self.get_default_joint_positions()
        return joint_positions
    
    def get_default_joint_positions(self):
        return [00.0, 30.0, 60.0] * 6

    def publish_joint_positions(self, joint_positions):
        for leg_id, position in enumerate (joint_positions):
            msg = LegCommand()
            msg.leg_id = leg_id
            msg.hip_angle = position[0]
            msg.knee_angle = position[1]
            msg.ankle_angle = position[2]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published joint positions for leg {leg_id}: {position}")




def main(args = None):
    rclpy.init(args = args)
    node = GaitControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()