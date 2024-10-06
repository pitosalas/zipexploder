#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Description video here
# https://drive.google.com/file/d/1H56bsOTd6OsTRQGtB25iiJXPFlWw5Ci-/view?usp=sharing

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        # Robot movement parameters
        self.linear_speed = 0.2  # Speed to move forward
        self.angular_speed = 0.3  # Speed to rotate

        # Initialize Twist message
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        # Current LIDAR data
        self.regions = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }

        # Initialize variables for LIDAR configuration
        self.angle_min = None
        self.angle_increment = None

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            rospy.loginfo(f"LIDAR Parameters: angle_min={self.angle_min}, angle_max={msg.angle_max}, angle_increment={self.angle_increment}")

        # Calculate indices for front, left, and right regions
        # Front region: -15 to +15 degrees
        # Left region: +45 to +90 degrees
         # Right region: -45 to -90 degrees
        angle_front_min = -15 * (math.pi / 180)  
        angle_front_max = 15 * (math.pi / 180)
        index_front_min = int((angle_front_min - self.angle_min) / self.angle_increment)
        index_front_max = int((angle_front_max - self.angle_min) / self.angle_increment)

        angle_left_min = 45 * (math.pi / 180)  
        angle_left_max = 90 * (math.pi / 180)
        index_left_min = int((angle_left_min - self.angle_min) / self.angle_increment)
        index_left_max = int((angle_left_max - self.angle_min) / self.angle_increment)

        angle_right_min = -90 * (math.pi / 180) 
        angle_right_max = -45 * (math.pi / 180)
        index_right_min = int((angle_right_min - self.angle_min) / self.angle_increment)
        index_right_max = int((angle_right_max - self.angle_min) / self.angle_increment)

        # Update regions with minimum distances
        self.regions['front'] = min([msg.ranges[i] for i in range(index_front_min, index_front_max + 1) if msg.ranges[i] < float('inf')] or [float('inf')])
        self.regions['left'] = min([msg.ranges[i] for i in range(index_left_min, index_left_max + 1) if msg.ranges[i] < float('inf')] or [float('inf')])
        self.regions['right'] = min([msg.ranges[i] for i in range(index_right_min, index_right_max + 1) if msg.ranges[i] < float('inf')] or [float('inf')])

    def find_wall(self):
        """

        Function to find the wall and align the robot's front to face the wall.
        Handles both outer and inner corners.
        """
        rospy.loginfo("Finding the wall...")

        safe_wall_distance = 0.5  # Distance to consider the wall "found"
        rate = rospy.Rate(10) 

        # Move forward until a wall is detected in front, left, or right
        while not rospy.is_shutdown():
            front_distance = self.regions['front']
            left_distance = self.regions['left']
            right_distance = self.regions['right']

            rospy.loginfo(f"Distances: Front={front_distance}, Left={left_distance}, Right={right_distance}")

            # Check if the wall is detected within threshold distance
            if front_distance <= safe_wall_distance or left_distance <= safe_wall_distance or right_distance <= safe_wall_distance:
                rospy.loginfo("Wall found. Preparing to align with the wall.")
                self.cmd.linear.x = 0.0  # Stop forward movement
                self.cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd)
                break
            else:
                rospy.loginfo("Moving forward to find a wall...")
                self.cmd.linear.x = self.linear_speed
                self.cmd.angular.z = 0.0  # Move straight

            self.cmd_vel_pub.publish(self.cmd)
            rate.sleep()

        # Rotate to face the wall considering both inner and outer corners
        rospy.loginfo("Rotating to face the wall...")
        while not rospy.is_shutdown():
            front_distance = self.regions['front']
            left_distance = self.regions['left']
            right_distance = self.regions['right']

            if front_distance > safe_wall_distance:
                # Rotate based on the wall position
                if left_distance < safe_wall_distance and right_distance > safe_wall_distance:
                    # Outer corner (Wall on the left)
                    rospy.loginfo("Outer corner detected, rotating left to face the wall...")
                    self.cmd.angular.z = self.angular_speed
                elif right_distance < safe_wall_distance and left_distance > safe_wall_distance:
                    # Outer corner (Wall on the right)
                    rospy.loginfo("Outer corner detected, rotating right to face the wall...")
                    self.cmd.angular.z = -self.angular_speed
                elif right_distance < safe_wall_distance and left_distance < safe_wall_distance:

                    # Inner corner (Wall on both sides)
                    rospy.loginfo("Inner corner detected, choosing direction...")
                    if right_distance < left_distance:                      
                        rospy.loginfo("Inner corner detected, rotating left to face the wall...")
                        self.cmd.angular.z = self.angular_speed
                    else:                    
                        rospy.loginfo("Inner corner detected, rotating right to face the wall...")
                        self.cmd.angular.z = -self.angular_speed
                else:
                    rospy.loginfo("Aligning to the wall.")
                    self.cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.cmd)
                    break
            else:
                rospy.loginfo("Aligned with the wall directly in front.")
                self.cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd)
                break

            self.cmd.linear.x = 0.0  
            self.cmd_vel_pub.publish(self.cmd)
            rate.sleep()

        rospy.loginfo("Wall finding and alignment process complete.")



    def follow_wall_clockwise(self):
        rospy.loginfo("Starting to follow the wall clockwise...")
        safe_distance_right = 0.5
        safe_distance_front = 0.8
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            front_distance = self.regions['front']
            right_distance = self.regions['right']

            rospy.loginfo(f"Distances: Front={front_distance}, Right={right_distance}")

            if front_distance < safe_distance_front:
                rospy.loginfo("Wall ahead, turning left...")
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.angular_speed
            elif right_distance > safe_distance_right:
                rospy.loginfo("Too far from the wall, turning right...")
                self.cmd.linear.x = self.linear_speed
                self.cmd.angular.z = -self.angular_speed
            else:
                rospy.loginfo("Following the wall...")
                self.cmd.linear.x = self.linear_speed
                self.cmd.angular.z = 0.0

            self.cmd_vel_pub.publish(self.cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        rospy.sleep(1)
        
        wall_follower.find_wall()
        wall_follower.follow_wall_clockwise()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

