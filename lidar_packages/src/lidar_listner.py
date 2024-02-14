#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan #commonly used to  represent data from LiDAR sensors.

def analyze_distances(ranges):
    obstacles = []
    for i, distance in enumerate(ranges):
        if distance < 1.0: #Lets consider 1.0m as threshold
            obstacles.append((i, distance))
    return obstacles

def filter_data(ranges):  #Makes the LiDAR data smooth
    filtered_ranges = []
    window_size = 5
    for i in range(len(ranges)):
        start = max(0, i - window_size // 2)
        end = min(len(ranges), i + window_size // 2)
        avg_range = sum(ranges[start:end]) / (end - start)
        filtered_ranges.append(avg_range)
    return filtered_ranges

def lidar_callback(msg):
    rospy.loginfo("LiDAR ranges: {}".format(msg.ranges))
    
    obstacles = analyze_distances(msg.ranges)
    rospy.loginfo("Detected obstacles: {}".format(obstacles))
    
    filtered_ranges = filter_data(msg.ranges)
    rospy.loginfo("Filtered LiDAR ranges: {}".format(filtered_ranges))

def lidar_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_listener()
    except rospy.ROSInterruptException:
        pass
