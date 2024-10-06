#!/usr/bin/env python3


#ROBOTICS VIDEO LINK
# https://drive.google.com/file/d/1vXzxCCrb1HCdDxmcbQ2kn8QBujVsvuCP/view?usp=sharing

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Constants
dista_from_wall = 0.5  
max_lidar_dista = 3.5  
min_lidar_dista = 0.1  
speed = 0.15           
spin = 0.2  
tolerance = 0.05  

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        self.twist = Twist()
        self.found_wall = False
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.back_distance = None
        self.side = 0  # -1 for right, 1 for left
        self.side_distance = None

    def scan_cb(self, msg):
        """
        Callback function for self.scan_sub.
        """
        lidar_ranges = msg.ranges
        front_ranges = lidar_ranges[358:] + lidar_ranges[:2]
        left_ranges = lidar_ranges[30:100] 
        right_ranges = lidar_ranges[260:330] 
        # right = cleaned_ranges[240:330] 
        # left = cleaned_ranges[30:120] 
        back_ranges = lidar_ranges[165:195] 

        # cleaned_ranges = [r for r in lidar_ranges if min_lidar_dista <= r <= max_lidar_dista]

        front = [r for r in front_ranges if min_lidar_dista <= r  and r != 'inf']
        left = [r for r in left_ranges if min_lidar_dista <= r <= max_lidar_dista]
        right = [r for r in right_ranges if min_lidar_dista <= r <= max_lidar_dista]
        # right = cleaned_ranges[240:330] 
        # left = cleaned_ranges[30:120] 
        back = [r for r in back_ranges if min_lidar_dista <= r and r != 'inf']

        # self.front_distance = self.calculate_mean(front)
        #make sure that front knows when it has none
        self.front_distance = min(front) if len(front) > 0 else None
        # self.left_distance = self.calculate_mean(left) works very well inside of the box
        self.left_distance = min(left) if len(left) > 0 else None
        self.right_distance = min(right) if len(right) > 0 else None
        # self.back_distance = self.calculate_mean(back)  # Distance behind the robot
        self.back_distance = min(back) if len(back) > 0 else None

        if self.side == -1:  # right
            self.side_distance = self.right_distance
        elif self.side == 1:  # left
            self.side_distance = self.left_distance

        print(f"side chosen: {self.side}")
    # def calculate_mean(self, data):
    #     """Helper function to calculate the mean of valid LIDAR values."""
    #     valid_values = [d for d in data if d is not None]
    #     if len(valid_values) == 0:
    #         return None 
    #     return sum(valid_values) / len(valid_values)

 
# have to fix when robot far away from left wall, so the first while loop has the issue of 
# directly moving front even tho if it detects a left but not right, or right but not left, it should 
# try to move towards that direction that it detects. 
# also need to fix when the robot is in front of the wall, far away, that it goes back because right now it does not detect 
# that there is a back distance as it is None and the front is currently detecting something even though there is nothing in front of it
# robot currently only works inside the box, on the close left of the box, and on the bottom of the boxn
    
    #conclusion:
    #once the robot correctly detects left as none, front as none, right and back as smth when outside of the box it should work
    # the other key thing is that if the robot is far away from the box, but it is already parallel to it,
    # it should move in that direction within align with wall, turn for a bit and move towards it, if the other direction is None.
    
    def align_with_wall(self):
        """
        Align with the nearest wall using LIDAR. The robot will stop aligning when
        it detects that it is parallel to the wall based on consistent side distances.
        """
        rate = rospy.Rate(10)
        print("Searching for the nearest wall...")

        #min(lidar range) => closest point to robot (not zero or inf or none)
        # look at front range, check front range to make it equal to min distance from robot
        #

        
        while (
            (self.right_distance is None or self.right_distance > 2 * dista_from_wall) and
            (self.left_distance is None or self.left_distance > 2 * dista_from_wall)
        ):
            # noving towards front
            print(f"right_dist:{self.right_distance} left_dist:{self.left_distance} back_dist: {self.back_distance} frontdist: {self.front_distance}")
            if (self.front_distance is not None) and (self.back_distance is None or (self.front_distance < self.back_distance)):
                print("No side walls detected, moving forward")
                #maybe include something here where if right dist has smth it turns in that direction slightly else left
                self.twist.linear.x = speed  # Move forward
                self.twist.angular.z = 0.0
            # moving towards back
            elif (self.back_distance is not None) and (self.front_distance is None or(self.back_distance < self.front_distance)):
                print("No side walls detected, moving backward")
                self.twist.linear.x = -speed  # Move backward
                #maybe include something here where if right dist has smth it turns in that direction slightly else left

                self.twist.angular.z = 0.0
            else:
                print("No walls detected. Rotating to search...")
                self.twist.angular.z = spin / 2  # Spin to search for a wall
                self.twist.linear.x = 0.0

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

        # choose closest wall
        if self.left_distance is not None and self.right_distance is not None:
            if self.left_distance < self.right_distance:
                self.side = 1  # left
                self.side_distance = self.left_distance
                print("CHOSEN< IM THE CHOSE ONE HERE YIPPEEEEELeft")
            else:
                self.side = -1  #right
                self.side_distance = self.right_distance
                print("CHOSEN< IM THE CHOSE ONE HERE YIPPEEEEERright")
        elif self.left_distance is not None:
            self.side = 1 
            self.side_distance = self.left_distance
            print(f" left distance{self.left_distance}")
            print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
            print(f" right distance{self.right_distance}")
        elif self.right_distance is not None:
            self.side = -1  
            self.side_distance = self.right_distance
            print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
            print(f" right distance{self.right_distance}")

        print(f"Aligning with the wall on {'right' if self.side == -1 else 'left'}.")

        # Variables to check for stable alignment
        stable_count = 0  
        stability_threshold = 5 
        previous_side_distance = None

        # Rotate in place to align the robot parallel to the wall
        while stable_count < stability_threshold:
            current_side_distance = self.left_distance if self.side == 1 else self.right_distance

            # Check if the distance is stable within tolerance
            if previous_side_distance is not None and abs(current_side_distance - previous_side_distance) < tolerance:
                stable_count += 1
                print(f"Stable count: {stable_count} (distance difference: {abs(current_side_distance - previous_side_distance)})")
            else:
                stable_count = 0  # Reset if the distance isn't stable

            previous_side_distance = current_side_distance

            self.twist.angular.z = spin  * self.side
            self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.found_wall = True  
        print(f"Wall alignment complete and wall found.side is: {self.side} ")




    def moving(self):
        """
        PD control loop to follow the wall while moving forward and turning corners.
        """
        rate = rospy.Rate(10)
        kp = .5
        kd = 4

        error = dista_from_wall - self.side_distance
        prev_error = error
        d_error = 0

        while not rospy.is_shutdown():
            

            if self.side_distance is not None:
                error = dista_from_wall - self.side_distance
                d_error = error - prev_error  if error - prev_error != 0 else d_error
                
                
                turn_amount = kp * error + kd * d_error
                
                
                angular_velocity = max(min(turn_amount * (-self.side), 0.4), -0.4) #rn it works for left

                # #try to make it work for right
                # if self.side == -1:
                #     if error <0:
                #         angular_velocity = abs(turn_amount)
                #     else:
                #         angular_velocity = -abs(turn_amount)
                #     angular_velocity = max(min(turn_amount, 0.4), -0.4) 

                #for turning inside corners
                if self.front_distance < max_lidar_dista:
                    angular_velocity = angular_velocity*5
                # turning outside corners
                if self.front_distance is None and self.side_distance > dista_from_wall*1.5:
                    angular_velocity = (speed/dista_from_wall)*1.2

                # if self.front_distance is None:
                prev_error = error 

                self.twist.linear.x = speed
                self.twist.angular.z = angular_velocity
                print(f"turnamount: {turn_amount} angular velocity: {angular_velocity} error: {error} d:{d_error}")
                # d_error: {d_error} side_chose:{self.side} right_dist:{self.right_distance} left_dist:{self.left_distance} side_dist: {self.side_distance} frontdist: {self.front_distance}")

                self.cmd_vel_pub.publish(self.twist)

                # rospy.loginfo(f"Error: {error}, Derivative: {d_error}, Angular Velocity: {angular_velocity}")
            elif self.side_distance is None:

                angular_velocity = spin * (self.side)
                self.twist.angular.z = angular_velocity
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
                print("finding wall again")
            
            
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                break

            rate.sleep()

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.found_wall:
                self.align_with_wall()
            else:
                rospy.loginfo("Following the wall...")
                self.moving()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
