#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

class CommunicationNode:
    def __init__(self):
        rospy.init_node('communication_node', anonymous=True)

        # Publisher for human detection information
        self.human_detection_pub = rospy.Publisher("/human_detection", String, queue_size=10)

        # Subscriber for receiving location information
        self.location_sub = rospy.Subscriber("/human_location", Point, self.location_callback)

    def location_callback(self, msg):
        # Process received location information
        rospy.loginfo("Received location: x={}, y={}, z={}".format(msg.x, msg.y, msg.z))
        # Add your code here to handle the received location information

    def publish_detection_info(self, detection_info):
        # Publish human detection information
        self.human_detection_pub.publish(detection_info)

    def run(self):
        rospy.spin()

def main():
    communication_node = CommunicationNode()
    try:
        communication_node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
