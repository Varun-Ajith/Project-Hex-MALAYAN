#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from hexapod_interfaces.msg import DetectionResult
import cv2
import numpy as np
import os
from pathlib import Path
import torch  # For YOLO implementation
from typing import Tuple, List
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'models/yolov5s.pt'),
                ('confidence_threshold', 0.5),
                ('enable_night_vision', True),
                ('detection_frequency', 10.0),  # Hz
            ]
        )
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.enable_night_vision = self.get_parameter('enable_night_vision').value
        self.detection_freq = self.get_parameter('detection_frequency').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.load_model()
        
        # Create QoS profile for reliable image transfer
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Initialize subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Initialize publishers
        self.detection_pub = self.create_publisher(
            DetectionResult,
            '/object_detection/detections',
            10
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            '/object_detection/debug_image',
            10
        )
        
        # Create timer for periodic detection
        self.timer = self.create_timer(1.0/self.detection_freq, self.detection_timer_callback)
        
        # Initialize state variables
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.last_detection_time = self.get_clock().now()
        
        self.get_logger().info('Object Detection Node initialized successfully')

    def load_model(self):
        """Load YOLO model for object detection"""
        try:
            # Load YOLOv5 model
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.conf = self.conf_threshold
            self.model.classes = [0]  # Only detect people
            if torch.cuda.is_available():
                self.model.cuda()
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            rclpy.shutdown()

    def image_callback(self, msg: Image):
        """Callback for receiving images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.frame_lock:
                self.current_frame = cv_image
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def enhance_night_vision(self, image: np.ndarray) -> np.ndarray:
        """Enhance image for night vision scenarios"""
        try:
            # Convert to LAB color space
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Apply CLAHE to L channel
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            cl = clahe.apply(l)
            
            # Merge channels
            limg = cv2.merge((cl,a,b))
            
            # Convert back to BGR
            enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
            
            return enhanced
        except Exception as e:
            self.get_logger().warn(f'Night vision enhancement failed: {str(e)}')
            return image

    def detect_objects(self, image: np.ndarray) -> List[Tuple[float, float, float, float, float, str]]:
        """Perform object detection on the image"""
        try:
            # Enhance image if night vision is enabled
            if self.enable_night_vision:
                image = self.enhance_night_vision(image)
            
            # Run inference
            results = self.model(image)
            
            # Process results
            detections = []
            for *xyxy, conf, cls in results.xyxy[0]:
                if conf > self.conf_threshold:
                    x1, y1, x2, y2 = map(float, xyxy)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    detection_type = "person" if int(cls) == 0 else "unknown"
                    detections.append((x1, y1, x2, y2, float(conf), detection_type))
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')
            return []

    def publish_detection(self, detection: Tuple[float, float, float, float, float, str], 
                         image: np.ndarray, image_height: int, image_width: int):
        """Publish detection results"""
        try:
            x1, y1, x2, y2, conf, det_type = detection
            
            # Create detection message
            det_msg = DetectionResult()
            det_msg.header.stamp = self.get_clock().now().to_msg()
            det_msg.header.frame_id = "camera_link"
            
            # Calculate center point
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Normalize coordinates to [-1, 1] range
            norm_x = (center_x / image_width) * 2 - 1
            norm_y = (center_y / image_height) * 2 - 1
            
            det_msg.location = Point(x=float(norm_x), y=float(norm_y), z=0.0)
            det_msg.confidence = float(conf)
            det_msg.detection_type = det_type
            
            # Extract ROI
            roi = image[int(y1):int(y2), int(x1):int(x2)]
            det_msg.roi = self.bridge.cv2_to_imgmsg(roi, "bgr8")
            
            # Publish detection
            self.detection_pub.publish(det_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detection: {str(e)}')

    def detection_timer_callback(self):
        """Timer callback for periodic detection"""
        with self.frame_lock:
            if self.current_frame is None:
                return
            
            frame = self.current_frame.copy()
        
        try:
            # Get image dimensions
            height, width = frame.shape[:2]
            
            # Perform detection
            detections = self.detect_objects(frame)
            
            # Process each detection
            for detection in detections:
                self.publish_detection(detection, frame, height, width)
                
                # Draw detection on debug image
                x1, y1, x2, y2, conf, det_type = detection
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'{det_type}: {conf:.2f}', 
                           (int(x1), int(y1-10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection timer callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Node error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()