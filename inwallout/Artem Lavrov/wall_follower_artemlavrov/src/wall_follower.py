#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

# https://drive.google.com/file/d/1-ptO_MCKIPAPmZT9lAdpTa-Vb7MeAw15/view?usp=sharing

# Constants
TARGET_DIST = 0.5
MAX_VEL = 0.1
PROP_CONST = 2
DERIV_CONST = 0.02
WANDERING_ANG_VEL = 0.5


class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.wall_data_sub = rospy.Subscriber(
            '/wall_data', Point, self.wall_data_cb)

        self.rate = rospy.Rate(10)
        self.ranges = None

        self.dist = None
        self.angle = None
        self.wall_detection_range = 0

    def scan_cb(self, msg):
        """
            Callback function for lidar scanner
        """
        self.ranges = [i if i <
                       msg.range_max and i > msg.range_min else -1
                       for i in msg.ranges]

    def wall_data_cb(self, msg):
        """
            Callback funciton to get wall data
        """
        self.angle = msg.x
        self.dist = msg.y
        self.wall_detection_range = msg.z

    def calc_pid(self):
        """
            PID function to calculate angular velocity
            based on distance to the wall and angle relative to the wall
        """
        dist_err = self.dist - TARGET_DIST
        deriv_err = -1 * self.angle

        pid = (dist_err * PROP_CONST + deriv_err * DERIV_CONST) * -1
        return pid

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """

        vel_twist = Twist()
        vel_twist.linear.x = MAX_VEL
        vel_twist.angular.z = self.calc_pid()
        self.cmd_vel_pub.publish(vel_twist)
        self.rate.sleep()

        vel_twist = Twist()
        self.cmd_vel_pub.publish(vel_twist)

    def find_wall(self):
        """
            Navigate until in range of the closest wall and rotate
            so that the wall is to the right of the robot
        """
        vel_twist = Twist()

        ranges = self.ranges
        min_dist = min(r for r in ranges if r > 0)
        min_index = ranges.index(min_dist)
        len_ranges = len(ranges)

        if min_index > len_ranges / 2:
            min_index = (min_index % (len_ranges / 2)) - (len_ranges / 2)

        if min_index < 15 or min_index > -15:
            vel_twist.linear.x = MAX_VEL
        else:
            vel_twist.linear.x = 0

        # If too far from the wall, rotate towards the wall and move closer
        # If within range of the wall but at wrong angle, rotate
        if min_dist > self.wall_detection_range * 0.9:
            vel_twist.angular.z = WANDERING_ANG_VEL * \
                (min_index / (len_ranges / 2))
        else:
            vel_twist.angular.z = -WANDERING_ANG_VEL
            vel_twist.linear.x = 0

        self.cmd_vel_pub.publish(vel_twist)

    def run(self):
        # Either follow wall or search for wall
        while not rospy.is_shutdown():
            # If no data is being received from wall data, find the wall
            if (self.dist is not None) and self.dist > 0:
                self.follow_wall()
            elif self.ranges is not None:
                # Find wall
                self.find_wall()
            else:
                pass
                # print("Waiting for data")


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().run()
