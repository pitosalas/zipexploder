#!/usr/bin/env python3
# https://drive.google.com/file/d/1YRrHIHrGt0JMLgfyDz8B3uYBRZary9zx/view?usp=sharing
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        #Constants used in code
        self.target_dist = 0.3
        self.twist = Twist()
        self.forward_dist = 0
        self.left_dist = 0
        self.right_dist = 0
        #These are to generalize the side 
        self.side = None
        self.side_dist = None

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        #Calculating distance from wall in front, to the left, and to the right
        degrees = [x for x in range(360) if x not in range(6, 355)]
        forward_ranges = list(filter(lambda x: x != float('inf'), [msg.ranges[x] for x in degrees]))
        left_ranges = list(filter(lambda x: x!= float('inf'), [msg.ranges[x] for x in range(0, 180)]))
        right_ranges = list(filter(lambda x: x!= float('inf'), [msg.ranges[x] for x in range(180, 360)]))
        self.forward_dist = min(forward_ranges) if len(forward_ranges) > 0 else 0
        self.left_dist = min(left_ranges) if len(left_ranges) > 0 else 0
        self.right_dist = min(right_ranges) if len(right_ranges) > 0 else 0

        #If a side has not been selected, choose a side
        if self.side is None:
            if self.left_dist != 0 and self.right_dist != 0:
                self.side = -1 if self.left_dist < self.right_dist else 1
            elif self.left_dist == 0:
                self.side = 1
            elif self.right_dist == 0:
                self.side = -1
        # The sides are -1 and 1 so I can multiply the PID output to account for 
        # how the robot will react in the opposite way depending on the side

        #Setting the side distance constant
        self.side_dist = self.left_dist if self.side == -1 else self.right_dist

    #Behavior when seeking a wall (when the robot is far away)
    def wall_seeking(self):

        self.twist.linear.x = 0.1
        rate = rospy.Rate(10)

        #PID constants
        kp = 0.05
        ki = 0
        kd = 0
        
        #Initializing error
        wall_error = self.target_dist - self.side_dist
        total_error = 0
        prev_error = wall_error
        d_error = 0

        #While far from wall
        while(wall_error < -0.65):
            #update error
            wall_error = self.target_dist - self.side_dist
            #update delta error
            total_error = total_error + wall_error
            d_error = wall_error - prev_error if wall_error - prev_error != 0 else d_error
            #change velocity to new velocity with formula
            vel = (wall_error* kp + d_error * kd + total_error * ki) * self.side
            self.twist.angular.z = vel

            #Publish to cmd_vel topic
            self.cmd_vel_pub.publish(self.twist)
            #update current degrees and error
            prev_error = wall_error
            rate.sleep()

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """

        #While the wall isn't found, spin
        while(self.side_dist == 0 or self.side_dist is None):
            self.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(self.twist)

        #When the wall is found but far away
        while self.target_dist - self.side_dist < -0.65:
            self.wall_seeking()

        self.twist.linear.x = 0.1
        rate = rospy.Rate(10)

        #PID constants
        kp = 1
        ki = 0
        kd = 10
        
        #Initializing error
        wall_error = self.target_dist - self.side_dist
        front_error = 0 if self.forward_dist == 0 else  1 / ((self.forward_dist + 0.85) ** 10)
        total_error = 0
        prev_error = wall_error
        d_error = 0

        while(abs):
            #update error
            wall_error = self.target_dist - self.side_dist
            front_error = 0 if self.forward_dist == 0 else  1 / ((self.forward_dist + 0.85) ** 10)
            #update delta error
            total_error = total_error + wall_error
            d_error = wall_error - prev_error if wall_error - prev_error != 0 else d_error
            #change velocity to new velocity with formula
            vel = ((wall_error + front_error)* kp + d_error * kd + total_error * ki) * self.side
            self.twist.angular.z = vel

            #Publish to cmd_vel topic
            self.cmd_vel_pub.publish(self.twist)
            #update current degrees and error
            prev_error = wall_error
            rate.sleep()
        
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
