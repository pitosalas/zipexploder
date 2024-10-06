#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Video Link - https://drive.google.com/file/d/1qoaI_UokCy3kXU0DYBxWnZJzz7hIReTK/view?usp=sharing

# Max linear speed the robot is allowed to reach
MAX_SPEED = 0.3
# Distance to be maintained from wall
DIST_FROM_WALL = 1
# Proportional constant for distance error
KP = 1.5
# Derivative constant for distance error
KD = 0.5
# Proportional constant for angular error
KPA = 1

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # Linear Velocity
        self.v = 0
        # Angular Velocity
        self.w = 0
        self.prev_dist_error = 0

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        # By default drive at MAX_SPEED
        self.v = MAX_SPEED
        # Get all distances between -10 degrees and 10 degrees
        front_distances = [r for r in (msg.ranges[:10] + msg.ranges[350:]) if not(math.isnan(r)) and not(math.isinf(r)) and r > msg.range_min and r < msg.range_max]
        if len(front_distances) != 0:
            # Calculate average front distance
            front_dist = sum(front_distances) / len(front_distances)
            # Stop if very close to a wall
            if front_dist < 0.1 * DIST_FROM_WALL:
                self.v = 0
            # Slow down if close to wall
            elif front_dist < DIST_FROM_WALL:
                self.v = 0.6 * MAX_SPEED

        # Scan all distances on the right side of the robot
        # The right side is divided into zones, each with the same zone_size
        # For example - the 181 degree zone covers 179 to 183 degrees given a zone_size of 4
        # In total, given a zone_size of 4, there will be 44 zones measured ranging from 181 degrees to 357 degrees
        shortest_dist_angle = 181
        zone_size = 4
        current_angle = 181
        while current_angle < 360:
            # current_dist stores the average of distances in the zone
            current_dist = 0
            # Number of valid angles in the zone
            angles_counted = 0
            for angle in range(current_angle - int(zone_size/2), current_angle + int(zone_size/2)):
                if angle > 359:
                    break
                # Skip if distance at this angle is invalid
                if math.isnan(msg.ranges[angle]) or math.isinf(msg.ranges[angle]) or msg.ranges[angle] < msg.range_min or msg.ranges[angle] > msg.range_max:
                    continue
                # Calculate the sum of distances in each zone
                current_dist += msg.ranges[angle]
                angles_counted += 1
            if angles_counted != 0:
                # Find average distance in the zone
                current_dist /= angles_counted
                # Find the zone angle with the lowest average distance
                if current_dist < msg.ranges[shortest_dist_angle]:
                    shortest_dist_angle = current_angle
            current_angle += zone_size
        
        # Find the lowest distance
        shortest_dist = msg.ranges[shortest_dist_angle]
        # Convert zone angle to radians
        shortest_dist_angle_rad = shortest_dist_angle * math.pi / 180
        # Shift angle by -2pi because ROS uses angles from pi to -pi
        shortest_dist_angle_rad -= 2 * math.pi
        # Calculate distance error, derivative of distance error, and angular error needed for PD controller
        dist_error = DIST_FROM_WALL - shortest_dist
        dist_derivative = dist_error - self.prev_dist_error
        angular_error = shortest_dist_angle_rad - ((-math.pi)/2)
        self.prev_dist_error = dist_error
        # PD controller using both distance and angle error
        self.w = KP * dist_error + KD * dist_derivative + KPA * angular_error

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = self.w
            twist.linear.x = self.v
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
