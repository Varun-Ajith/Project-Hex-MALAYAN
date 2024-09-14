import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorDataNode(Node):
    def __init__(self):
        super().__init__("sensor_input_node")

        self.publisher_ = self.create_publisher(
            String,
            'sensor_input',
            10
        )
        time_period_ = 2
        self.timer_ = self.create_timer(time_period_, self.publish_sensor_input_)
    
    def publish_sensor_input(self):
        msg = String()
        msg.data = "survivor_detected"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing : {msg.data} to sensor_input")

def main(args = None):
    rclpy.init(args = args)
    node = SensorDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()