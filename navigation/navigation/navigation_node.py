import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        self.declare_parameter("navigation_command_topic", "navigation_command")
        self.navigation_command_topic_ = self.get_parameter("navigation_command_topic").get_parameter_value().string_value

        self.command_subscriber_ = self.create_subscription(
            String,
            self.navigation_command_topic_,
            self.command_callback_,
            10
        )
    
    def command_callback_(self, msg):
        if msg.data == "move to survivor":
            self.get_logger().info("Moving towards detected survivor...")
        else:
            self.get_logger().info("Stand by!")
    
def main(args = None):
    rclpy.init(args = args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    