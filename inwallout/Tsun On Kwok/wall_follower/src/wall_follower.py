#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
import numpy as np

# https://drive.google.com/file/d/148CYzIS4NjwTWNwsQL5zJs71xqbKbFht/view?usp=sharing

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.rate = rospy.Rate(10)

        self.twist = Twist()
        self.cloest_wall = None
        self.cloest_wall_angle = None
        self.target_dist = 1.0

    def scan_cb(self, msg):
        ranges = msg.ranges

        # Split the ranges into 30 parts and the angle map of each part
        part_deg = 12
        split_ranges = [ranges[-6:] + ranges[:6]] + [ranges[i: i + part_deg] for i in range(6, 354, part_deg)]
        angle_map = [(2 * i + part_deg) // 2 for i in range(-6, 354, part_deg)]

        # Calculate the mean distance for each part, ignoring values > 5 and < 0.1
        mean_distances = []
        for part in split_ranges:
            valid_ranges = [r for r in part if 0.1 <= r <= 5]
            if valid_ranges:
                mean_distances.append(np.mean(valid_ranges))
            else:
                mean_distances.append(float('inf'))  # Use infinity if no valid ranges
        self.cloest_wall = min(mean_distances)
        self.cloest_wall_angle = angle_map[mean_distances.index(self.cloest_wall)]

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
    
    # optional: try to find wall from any location and move to parallel to the wall
    def move_to_start_postion(self):
        while self.cloest_wall_angle is None:
            rospy.logwarn("Current lidar is not available. Waiting...")
            self.rate.sleep()
        
        # turn to face the closest wall
        self.twist.angular.z = -0.4 if self.cloest_wall_angle > 180 else 0.4
        while self.cloest_wall_angle != 0:
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.stop()

        # move to postion with target distance between the cloest wall
        while not rospy.is_shutdown() and self.cloest_wall > self.target_dist:
            self.twist.linear.x = 0.4
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.stop()
        
        # turn to parallel to the wall
        while self.cloest_wall_angle != 96:
            self.twist.angular.z = 0.4 
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.stop()

    # following along a wall on the left side of the robot
    def following_wall(self):
        while self.cloest_wall is None:
            rospy.logwarn("Current angle is not available. Waiting...")
            self.rate.sleep()
        
        # control the robot with PID
        Kp = 0.8
        Ki = 0.0  # I acutally dont use the intergral as I found it not very helpful
        Kd = 10

        previous_error = 0
        integral = 0
        while not rospy.is_shutdown():
            error = self.target_dist - self.cloest_wall
            integral += error
            derivative = error - previous_error
            previous_error = error
        
            control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # a slow_factor such the linear velocity decrease when angular velocity decrease
            slow_factor = 1 / (1 + abs(control_signal))

            self.twist.linear.x = 0.4 * slow_factor
            self.twist.angular.z = -control_signal
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

    def follow_wall(self):
        self.move_to_start_postion()
        self.following_wall()
    
if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()