#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class GaitMovementNode:
    def __init__(self):
        rospy.init_node('gait_movement_node', anonymous=True)
        self.rate = rospy.Rate(10)  # Control loop rate in Hz
        self.leg_positions = [0.0, 0.0, 0.0] * 6  # Initial positions of leg servos

        # Publishers for servo positions
        self.servo_pubs = []
        for i in range(1, 7):
            self.servo_pubs.append(rospy.Publisher(f"/leg{i}_joint_controller/command", Float64, queue_size=10))

        # Subscriber for receiving velocity commands
        self.velocity_sub = rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)

    def velocity_callback(self, msg):
        # Calculate servo positions based on velocity commands (linear and angular)
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate leg positions for tripod gait
        t = rospy.Time.now().to_sec()
        for i in range(1, 7):
            leg_pos = math.sin(2 * math.pi * t + (i-1) * (2 * math.pi / 3)) * 0.5  # Example calculation, replace with actual gait pattern
            for j in range(3):
                self.servo_pubs[(i-1)*3 + j].publish(leg_pos)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

def main():
    movement_node = GaitMovementNode()
    movement_node.run()

if __name__ == '__main__':
    main()
