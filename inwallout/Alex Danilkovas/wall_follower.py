#!/usr/bin/env python3

# https://drive.google.com/file/d/1n69_CzrjC6yVtnx69JsyKt6cevC3qg-i/view?usp=sharing

import math
import rospy
import utils
import statistics as stats
from enum import Enum
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

DISTANCE_FROM_WALL = 0.5
IDEAL_LIN_VEL = 0.1

class Side(Enum):
    LEFT=0,
    RIGHT=1

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.rate = rospy.Rate(10)

        self.ranges = None
        self.closest = (None, None)  # (Dist, Angle)

        self.turning = False
        self.following_side = None

        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        rospy.loginfo("Robot stopped due to shutdown.")

    def scan_cb(self, msg):
        self.ranges = msg.ranges
        min_val = float('inf')
        min_index = None

        for i, value in enumerate(msg.ranges):
            if value <= msg.range_min or value >= msg.range_max: continue
            if value >= min_val: continue

            min_val = value
            min_index = i

        if min_index is not None: self.closest = (min_val, min_index)
        else: self.closest = (None, None)



    def follow(self, sideList, side):
        avg = stats.mean(sideList)
        delta = avg - DISTANCE_FROM_WALL

        turn_indication_range = self.ranges[280:290] if side == Side.RIGHT else self.ranges[70:80]

        direction = -1 if side == Side.RIGHT else 1

        if stats.mean(turn_indication_range) > DISTANCE_FROM_WALL+1:
            self.turning = True
            # pid.clear()

            twist = Twist()
            twist.angular.z = direction * 0.1/DISTANCE_FROM_WALL
            twist.linear.x = 0.1
            t = time.time()
            while time.time() < t + ((math.pi * DISTANCE_FROM_WALL)/0.2): # (math.pi / 2) / 0.1
                self.cmd_vel_pub.publish(twist)


            t = time.time()
            twist.angular.z = 0.0
            twist.linear.x = 0.1
            while time.time() < t + (0.25 * DISTANCE_FROM_WALL)/IDEAL_LIN_VEL:
                self.cmd_vel_pub.publish(twist)

            self.turning = False


        twist = Twist()
        if self.closest[1] == None: return

        desired_follow_angle = 270 if side == Side.RIGHT else 90
        pid_output = pid.compute(desired_follow_angle, self.closest[1])

        twist.angular.z = -pid_output/1.5
        if delta > 0.1:
            twist.angular.z = -pid_output/1.5 + direction * 0.2
        elif delta < -0.1:
            twist.angular.z = -pid_output/1.5 - direction * 0.2
        

        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)


    def inside_turn(self, follow_side):
        self.turning = True
        direction = 1 if follow_side == Side.RIGHT else -1

        twist = Twist()
        twist.angular.z = direction * 0.1
        
        t = time.time()
        while time.time() < t + (math.pi / 2) / 0.1:  
            self.cmd_vel_pub.publish(twist)

        twist.angular.z = 0.0  
        self.cmd_vel_pub.publish(twist)
        
        self.turning = False



    def follow_wall(self):
        while not rospy.is_shutdown():
            if not self.ranges: continue
            if self.turning: continue

            left_side = self.ranges[80:100]  
            right_side = self.ranges[260:280] 
            front = self.ranges[350:360] + self.ranges[0:10]

            if stats.mean(front) < DISTANCE_FROM_WALL:  
                if not self.following_side: self.inside_turn(Side.RIGHT)
                else: self.inside_turn(self.following_side)

            elif stats.mean(left_side) < DISTANCE_FROM_WALL + 2:
                self.following_side = Side.LEFT
                self.follow(left_side, Side.LEFT)

            elif stats.mean(right_side) < DISTANCE_FROM_WALL + 2:  
                self.following_side = Side.RIGHT
                self.follow(right_side, Side.RIGHT)


            else:
                twist = Twist()
                twist.linear.x = 0.1
                self.cmd_vel_pub.publish(twist)

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    pid = utils.PID(-0.2, 0.2, 10, 0.0, 0.1)
    follower = WallFollower()
    follower.follow_wall()
