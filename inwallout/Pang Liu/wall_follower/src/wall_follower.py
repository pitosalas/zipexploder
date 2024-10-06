#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
'''
Video Address:
https://drive.google.com/file/d/1AvAN7iIaqyN2XfOrDdxgwjHq7ksBSzno/view?usp=sharing
'''

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # When subscribing to new lidar data, ROS will automatically call the scan_cb() callback function and pass the latest lidar data (msg).
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        self.infinite_distance = 10.0 # Assume the maximum distance is 10.0

        self.dead_distance = 0.3 # This is the distance that needs special algorithm
        self.keep_distance = 1.0 # The relative distance to be set (that is, the distance to be maintained)
        self.boundry_distance = 3.0 # The set boundary distance (that is, the definition of well away)

        self.angular_speed = 0.3 # Angular velocity, a positive value means counterclockwise rotation, that is, turning left
        self.angular_speed_slow = self.angular_speed * 0.3 # Slow rotation speed

        self.linear_speed = 0.25 # Linear velocity
        self.linear_speed_slow = self.linear_speed * 0.3 # Slow linear velocity

        self.closest_obstacle_dist = self.infinite_distance     # The closest distance
        self.closest_direction = 0                              # The direction at the closest distance

        self.foreward_distance = self.infinite_distance   # foreward distance
        self.left_distance = self.infinite_distance       # leftside distance
        self.backward_distance = self.infinite_distance   # backward distance 
        self.right_distance = self.infinite_distance      # rightside distance

    def scan_cb(self, msg):
        """
        The callback function
        """
        # The closest_obstacle_dist
        valid_closest_dist = min([r for r in msg.ranges if r != float('inf')]) if [r for r in msg.ranges if r != float('inf')] else self.infinite_distance
        self.closest_obstacle_dist = valid_closest_dist if valid_closest_dist != float('inf') else self.infinite_distance
        rospy.loginfo(f"Closest obstacle distance: {self.closest_obstacle_dist}") # Print closest obstacle distance

        # The closest_direction
        if self.closest_obstacle_dist < self.infinite_distance:
            self.closest_direction = msg.ranges.index(self.closest_obstacle_dist) 
            rospy.loginfo(f"Closest distance direction: {self.closest_direction}") # Print the direction in closest distance
        else:
            self.closest_direction = -1 # -1 means there is no closest direction

        # Four direction's distance
        # I try to use these variables to solve the inner corner problem, but didn't get a complete solution before the submission ddl...
        self.foreward_distance = self.get_distance_at_direction(msg, 0)     # 0 is foreward_distance
        self.left_distance = self.get_distance_at_direction(msg, 90)        # 90 is left_distance
        self.backward_distance = self.get_distance_at_direction(msg, 180)   # 180 is backward_distance
        self.right_distance = self.get_distance_at_direction(msg, 270)      # 270 is right_distance

    def get_distance_at_direction(self, msg, n):
        """
        Get the distance in n direction of the LiDAR scan
        """
        return msg.ranges[n] if msg.ranges[n] != float('inf') else self.infinite_distance

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        twist = Twist()
        
        # Condition < 0 >
        # Algorithm of following wall inner the dead zone (including inner corner)
        # Situation(6): The wall has an inside corner
        # THIS PART IS NOT FINISHED YET, WILL CONITNUE UPDATING AFTER SUBMITTING
        if self.closest_obstacle_dist < self.dead_distance:
            # This part is handling situations of too close to the wall and meet the inner corner
            # Detect this situation first, when detected, keep away from such situation
            twist.angular.z = self.angular_speed_slow
            twist.linear.x = self.linear_speed_slow

        # Condition < 1 >
        # Algorithm of following wall between dead line and keep line
        # Situation(1) Robot already at the right distance from the wall and pointing parallel to the wall
        # Situation(3) Robot at 0.5 the desired distance and pointing in the right direction
        # Situation(4) Robot is pointing in various other ways (inside target distance, face backward the wall)
        # Situation(7) The wall has an outside corner --- the virtual wall can be assumed, so no problem
        elif self.closest_obstacle_dist < self.keep_distance:
            if 0 <= self.closest_direction < 90:
                twist.angular.z = - self.angular_speed
                twist.linear.x = self.linear_speed_slow
            elif 270 <= self.closest_direction:
                twist.angular.z = self.angular_speed
                twist.linear.x = self.linear_speed_slow
            # Keep going, no turn
            elif 90 <= self.closest_direction < 135 or 225 <= self.closest_direction < 270:
                twist.angular.z = 0.0
                twist.linear.x = self.linear_speed
            # Keep turning, no move
            elif 135 <= self.closest_direction < 180:
                twist.angular.z = self.angular_speed
                twist.linear.x = self.linear_speed_slow
            elif 180 <= self.closest_direction < 225:
                twist.angular.z = - self.angular_speed
                twist.linear.x = self.linear_speed_slow

        # Condition < 2 >
        # Algorithm of following wall between keep line and boundry line
        # (2) Robot at 1.5 the desired distance and pointing in the right direction
        # (4) Robot is pointing in various other ways (outside target distance, face toward the wall)
        elif self.boundry_distance > self.closest_obstacle_dist > self.keep_distance:
            twist.linear.x = self.linear_speed
            if 90 <= self.closest_direction < 180:
                twist.angular.z = self.angular_speed
            elif 180<= self.closest_direction < 270:
                twist.angular.z = - self.angular_speed
            # Keep going, no turn
            elif 45 <= self.closest_direction < 90 or 270 <= self.closest_direction < 315:
                twist.angular.z = 0.0
                twist.linear.x = self.linear_speed
            # Keep rotating, no move
            elif 0 <= self.closest_direction < 45:
                twist.angular.z = - self.angular_speed
                twist.linear.x = 0.0
            elif 315 <= self.closest_direction:
                twist.angular.z = self.angular_speed
                twist.linear.x = 0.0  

        # Condition < 3 >
        # (5) Robot is well away from the wall 
        elif self.closest_direction == -1 or self.closest_obstacle_dist >= self.boundry_distance - 0.1:  # 0.1 is the dead zone avoid chaos
            # if there is no wall near the robot, draw big circle
            # if still no wall, draw another big circle (another direction)
            # if still no wall, draw bigger circle...
            # THIS PART IS NOT FINISHED YET, WILL CONITNUE UPDATING AFTER SUBMITTING
            twist.linear.x = 0.2 
            twist.angular.z = 0.3 
        
        self.cmd_vel_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    
    wallfollower = WallFollower()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        wallfollower.follow_wall()
        rate.sleep()






