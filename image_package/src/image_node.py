#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImageRecognitionNode:
    def __init__(self):
        rospy.init_node('image_recognition_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        
        # Load Haar cascade classifier for human detection
        self.human_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            return

        # Convert the image to grayscale for faster processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect humans in the image
        humans = self.human_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Publish detection results
        if len(humans) > 0:
            rospy.loginfo("Detected {} human(s)".format(len(humans)))
            for (x, y, w, h) in humans:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Display the annotated image (optional)
        cv2.imshow("Human Detection", cv_image)
        cv2.waitKey(1)

def main():
    recognition_node = ImageRecognitionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
