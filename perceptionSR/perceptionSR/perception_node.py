from rclpy.node import Node
import rclpy
from std_msgs.msg import String

class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")


        self.declare_parameter('survivor_detection_topic', "survivor_detection")

        self.survivor_detection_topic = self.get_parameter('survivor_detection_topic').get_parameter_value().string_value

        self.sensor_subscriber_ = self.create_subscription(
                                  String,
                                  "sensor_input",
                                  self.sensor_callback_,
                                  10
        )

        self.survivor_publisher_ = self.create_publisher(
                                    String,
                                    self.survivor_detection_topic,
                                    10
        )

    def sensor_callback_(self, msg):
        detection = String()
        if msg.data == "survivor detected":
            detection.data = "Survivor located"
        else:
            detection.data = "survivor not located"
        self.survivor_publisher_.publish(detection)
        self.get_logger().info(f"Publishing : {detection.data}")
def main(args = None):
    rclpy.init(args = args)
    perceptionNode = PerceptionNode()
    rclpy.spin(perceptionNode)
    perceptionNode.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()