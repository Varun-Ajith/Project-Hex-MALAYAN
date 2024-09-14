import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommunicationNode(Node):
    def __init__(self):
        super().__init__("communication_node")

        self.declare_parameter("communication_topic", "communication_topic")
        self.communication_topic_ = self.get_parameter("communication_topic").get_parameter_value().string_value
        self.status_subscriber_ = self.create_subscription(
            String,
            self.communication_topic_,
            self.status_callback_,
            10
        )
    def status_callback_(self, msg):
        self.get_logger().info(f"Replaying message to rescue team : {msg.data}")

def main(args = None):
    rclpy.init(args = args)
    node = CommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()