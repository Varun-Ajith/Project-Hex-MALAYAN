#! /usr/bin/env python3

import rclpy
from hexapod_perception.object_detection import ObjectDetectionNode
from hexapod_perception.sensor_fusion import SensorFusionNode

def main(args = None):
    rclpy.init(args = args)
    node1 = SensorFusionNode()
    rclpy.spin(node1)
    node2 = ObjectDetectionNode()
    rclpy.spin(node2)
    rclpy.shutdown()

if __name__ == "__main__":
    main()