import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")

        self.declare_parameter("survivor_detection_topic", "survivor_detection")
        self.declare_parameter("navigation_command_topic",  "navigation_command")
        self.declare_parameter("communication_topic", "communication_topic")

        self.get_parameter_values()

        self.callback_group = ReentrantCallbackGroup()

        self.survivor_subscriber = self.create_subscription(
            String,
            self.survivor_detection_topic_,
            self.survivor_callback_,
            10,
            callback_group = self.callback_group
        )
        self.navigation_publisher = self.create_publisher(
            String,
            self.navigation_command_topic_,
            10
        )
        self.communication_publisher_ = self.create_publisher(
            String,
            self.communication_topic_,
            10
        )
    
    def get_parameter_values(self):
        self.survivor_detection_topic_ = self.get_parameter("survivor_detection_topic").get_parameter_value().string_value
        self.navigation_command_topic_ = self.get_parameter("navigation_command_topic").get_parameter_value().string_value
        self.communication_topic_ = self.get_parameter("communication_topic").get_parameter_value().string_value

    def survivor_callback_(self,msg):
        if "Survivor located" in msg.data:
            command = String()
            command.data = "move to survivor"
            self.navigation_publisher.publish(command)
            self.get_logger().info("Commanding Navigation: move to survivor")

            communication_message = String()
            communication_message.data = "survivor located"
            self.communication_publisher_.publish(communication_message)
            self.get_logger().info("Notifying rescue team: survivor located")
        else:
            self.get_logger().info("No action required")

def main(args = None):
    rclpy.init(args = args)
    node = BrainNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()