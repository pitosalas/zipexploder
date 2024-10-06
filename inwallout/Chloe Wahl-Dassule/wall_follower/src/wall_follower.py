#!/usr/bin/env python3
import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#https://drive.google.com/file/d/1aehcrYiTwqlt7mChbYBPGIY-40VJuG9F/view?usp=sharing

DISTANCE = 1
STOP_DIST = 0.1
RATE = 30
ERROR =0.3
SPEED=0.1
VIEW= 0.25

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.dist_left = DISTANCE
        self.dist_front = 99.9
        self.twist = Twist()

        self.last_yaw = 0.0
        self.cur_yaw = 0.0
        self.dist = 0.0
        self.total_dist = 0.0
        self.start_bearing = 0.0
        self.range_max = 0.0
    
    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.last_yaw = self.cur_yaw
        self.cur_yaw = msg.y
        self.dist = msg.x
        self.total_dist += self.dist

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        ranges = msg.ranges

        self.range_max = msg.range_max
        
        #adjust based on current yaw from starting bearing
        percent_change = self.cur_yaw/(2*math.pi)
        
        #get the average distance to the left of the robot based on the current yaw
        left_min = int(len(ranges) * (0.25 - (VIEW / 2) - percent_change))
        left_max = int(len(ranges) * (0.25 + (VIEW / 2) - percent_change))
        print(f"left_min: {left_min} left_max: {left_max} length: {len(ranges)}")
        if left_min < 0 :
            left_min = len(ranges) + left_min
        if left_max < 0:
            left_max = len(ranges) + left_max
            
        #get the average
        if left_min < left_max:
            self.dist_left = self.get_dist_range(msg, left_min, left_max)
        else:
            self.dist_left = self.get_dist_range(msg, left_max, left_min)

        #get the average distance to the front of the robot
        front_min = int(len(ranges) * 0.10)
        front_max = int(len(ranges) * 0.90) 
        self.dist_front = self.get_dist_range(msg, front_min, front_max)

    def posify(self, yaw):
        #get the yaw in positive radians
        if yaw < 0:
            yaw = PI + (PI + yaw)
        return yaw
    
    def get_dist_range(self, msg, min, max):
        #get the average from cleaned data
        ranges = msg.ranges[min:max + 1]
        filtered_ranges = []
        for value in ranges:
            if value > msg.range_max or value < msg.range_min:
                filtered_ranges.append(np.nan)
            else:
                filtered_ranges.append(value)
        return np.nanmean(filtered_ranges)

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        rate = rospy.Rate(RATE)
        self.start_bearing = self.posify(self.cur_yaw)

        while not rospy.is_shutdown():

           
            distance = self.dist_left #how far from the wall
            max_yaw = math.pi / 4 #max yaw the robot can have
            max_angle = math.pi / 4 #max angle to turn at
            yaw_error = DISTANCE * 0.05 #how 
            hypotenuse = 7 #7-8 was pretty good
            error = DISTANCE - distance #error

            #determine speed
            if abs(error) >= ERROR:
                self.twist.linear.x =  SPEED
            else:
                self.twist.linear.x =  SPEED / 2


            #calculate the turn
            if abs(error) < hypotenuse:
                radians = math.asin(abs(error) / hypotenuse)
                if radians > max_angle:
                    randians = max_angle
            else:
                radians = max_angle
    
            
            if abs(error) <= yaw_error and abs(self.start_bearing - self.cur_yaw) > math.pi / 30:
                #turn the robot back to ints original yaw if it is on the line
                #used to dampen oscilation
                print(f"ON THE LINE {error} <= {yaw_error}")
                self.twist.linear.x = 0
                if self.cur_yaw < 0:
                    self.twist.angular.z = max_angle
                else:
                   self.twist.angular.z = - max_angle 
            elif abs(self.start_bearing - self.cur_yaw) >= (max_yaw):
                #rotate to be facing "forward" if the robot is too turned
                if self.cur_yaw < 0:
                    print("PERPENDICULAR -> too right")
                    self.twist.linear.x = 0
                    self.twist.angular.z = max_yaw
                    #too far to the right
                else:
                    #too far to the left
                    print("PERPENDICULAR -> too left")
                    self.twist.linear.x = 0
                    self.twist.angular.z = - max_yaw
            else:
                #turn the course correcting distance
                if distance >= DISTANCE:
                    #go to the left
                    self.twist.angular.z = radians
                else:
                    self.twist.angular.z = -radians
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
