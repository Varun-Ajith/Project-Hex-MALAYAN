#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):
    # Process LiDAR data here
    # For this example, let's just print the ranges
    rospy.loginfo("LiDAR ranges: {}".format(msg.ranges))

def lidar_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_listener()
    except rospy.ROSInterruptException:
        pass
