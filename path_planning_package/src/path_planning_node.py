#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pose = None
        self.scan = None

    def odom_callback(self, odom_msg): # Get robot's pose from odometry
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose = {
            'x': odom_msg.pose.pose.position.x,
            'y': odom_msg.pose.pose.position.y,
            'theta': yaw
        }

    def lidar_callback(self, scan_msg): # Receive LaserScan data
        self.scan = scan_msg

    def calculate_command(self):
        if self.scan is not None and self.pose is not None:
            linear_x = 0.3  # Forward velocity
            angular_z = 0.0  # Angular velocity

            obstacle_detected = False
            min_range = min(self.scan.ranges)
            if min_range < 0.5:  # If an obstacle is detected within 0.5 meters
                obstacle_detected = True

            if obstacle_detected:
                # Turn right
                angular_z = 0.5
            else:
                # Go straight
                angular_z = 0.0

            cmd_vel = Twist()
            cmd_vel.linear.x = linear_x
            cmd_vel.angular.z = angular_z
            self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.calculate_command()
            rate.sleep()

if __name__ == '__main__':
    path_planning_node = PathPlanningNode()
    path_planning_node.run()
