#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# https://drive.google.com/file/d/1qBzFI3dwxckYLj7r4gyQ8IIW4cpkp9-P/view?usp=sharing

hz = 10
threshold = 0.15 # threshold for ideal angular difference between left front and left back (if they are equal robot assumes it is parallel to wall)
ideal = .75 # ideal distance from wall
speed = 0.35
angular_speed = 0.35
parallel_angular_speed = 0.15
kd = 0.15 # for derivative

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.forward_distance = float('inf')
        self.left_back_distance = float('inf')
        self.left_forward_distance = float('inf')
        self.left_distance = float('inf')
        self.prev = 0

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """

        self.left_distance = self.average_no_inf(msg.ranges[80:100])  # Get average distance from left side of robot
        self.left_forward_distance = self.average_no_inf(msg.ranges[60:80])  # Get average distance from left front of robot
        self.left_back_distance = self.average_no_inf(msg.ranges[100:120])  # Get average distance from left back of robot

    def average_no_inf(self, range):
        """
        Gets the average distance from a certain range (not including inf values)
        """
        values = []
        for x in range:
            if x != float('inf') and x != float('-inf'):
                values.append(x)

        if len(values) == 0:
            return float('inf')
        else:
            return sum(values) / len(values) # includes nan values

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        rate = rospy.Rate(hz)
        twist = Twist()

        while not rospy.is_shutdown():
            if float('inf') in [self.left_forward_distance, self.left_distance, self.left_back_distance]:
                #has not found wall yet
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                d = self.left_forward_distance - self.left_back_distance
                
                error = self.left_distance - ideal
                derivative = error - self.prev
                self.prev = error
                twist.linear.x = speed / 2
                if d < -threshold: # facing towards so move out
                    twist.angular.z = -angular_speed * error - kd * derivative 
                elif d > threshold: # facing away - move in
                    twist.angular.z = angular_speed * error + kd * derivative # pid kind of
                else:
                    twist.angular.x = speed # go faster since parallel
                    twist.angular.z = parallel_angular_speed * error

            self.cmd_vel_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
