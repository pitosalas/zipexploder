#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
# https://drive.google.com/file/d/1UEchva6zNcz9YFYwC4OYspe7JyOkUaMd/view?usp=drive_link

class WallFollower:
    FIXED_DIST = 0.5
    SAFE_DIST = 0.5
    PI = 3.1416

    def __init__(self):
        rospy.init_node('wall_follower')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.range = None
        self.range_front = None
        self.range_right = None
        self.range_left = None
        self.min_front = float('inf')
        self.min_right = float('inf')
        self.min_left = float('inf')
        self.is_aligned = False
        self.is_close = False

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        Processes laser scan data to update robot movement.
        """
        self.range = msg.ranges
        # Define the desired wedges
        self.range_front = list(msg.ranges[-15:]) + list(msg.ranges[:15])
        self.range_right = msg.ranges[300:345]
        self.range_left = msg.ranges[15:60]
        # The data from the Lidar needs a little cleaning
        self.min_front = min([r for r in self.range_front if not math.isnan(r) and not math.isinf(r)], default=float('inf'))
        self.min_right = min([r for r in self.range_right if not math.isnan(r) and not math.isinf(r)], default=float('inf'))
        self.min_left = min([r for r in self.range_left if not math.isnan(r) and not math.isinf(r)], default=float('inf'))

    def follow_wall(self):
        """
        Makes the robot follow a wall using the latest laser scan data.
        """
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.is_close:
                self.move_to_wall()
            else:
                if not self.is_aligned:
                    if self.min_left < WallFollower.FIXED_DIST:
                        self.is_aligned = True
                        rospy.loginfo("Aligned with the wall.")
                    else:
                        twist.angular.z = -0.5  # To be parallel with the wall 
                        twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(twist)
                        rate.sleep()
                        continue
                
                if self.min_front < WallFollower.SAFE_DIST:
                    # Obstacle in front, turn right
                    twist.angular.z = -0.5
                    twist.linear.x = 0.0
                    rospy.loginfo("Obstacle detected in front. Turning right.")
                elif self.min_left < WallFollower.FIXED_DIST - 0.05:
                    # Too close to the wall, turn right
                    twist.angular.z = -0.5
                    twist.linear.x = 0.3
                    rospy.loginfo("Too close to the wall. Turning right.")
                elif WallFollower.FIXED_DIST - 0.05 <= self.min_left <= WallFollower.FIXED_DIST + 0.05:
                    # Correct distance, move forward
                    twist.angular.z = 0.0
                    twist.linear.x = 0.3
                    rospy.loginfo("Maintaining correct distance from the wall.")
                else:
                    # Too far from the wall, turn left
                    twist.angular.z = 0.5
                    twist.linear.x = 0.3
                    rospy.loginfo("Too far from the wall. Turning left.")

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def move_to_wall(self):
        """
        Makes the robot closer to the wall if it is too far away from the wall at the start.
        """
        min_value = float('inf')

        twist = Twist()
        rate = rospy.Rate(10)

        angular_speed = 0.5
        full_rotation_time = int(2 * WallFollower.PI / angular_speed * 10)

        start_time = rospy.Time.now()
        
        for i in range(full_rotation_time): # Turn around to find the wall
            twist.linear.x = 0.0
            twist.angular.z = -angular_speed

            if self.min_front < min_value:
                min_value = self.min_front
                rospy.loginfo("Turning around, finding the wall")

            if min_value < WallFollower.FIXED_DIST:
                self.is_close = True # already near the wall
                break

            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        rospy.loginfo("Wall found")

        while not rospy.is_shutdown() and not self.is_close: 
            if abs(self.min_front - min_value) < 0.02: # Travel to the wall
                twist.linear.x = 0.3
                twist.angular.z = 0.0

                target_dist = self.min_front - WallFollower.FIXED_DIST + 0.3
                travel_time = target_dist / twist.linear.x
                
                start_time = rospy.Time.now()

                while(rospy.Time.now() - start_time).to_sec() < travel_time:
                    self.cmd_vel_pub.publish(twist)
                    rospy.loginfo("Getting closer to the wall")

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)
                self.is_close = True
            else: # Finding the wall
                twist.linear.x = 0.0
                twist.angular.z = -angular_speed
                self.cmd_vel_pub.publish(twist)
                
            rate.sleep()

            
if __name__ == '__main__':
    wall_follower = WallFollower()
    wall_follower.follow_wall()
    rospy.spin()
