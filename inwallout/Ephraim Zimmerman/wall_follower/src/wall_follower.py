#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# https://drive.google.com/file/d/18gei9M4Cz7bO8n5lRa0DcmFkw4zfOTvx/view?usp=sharing

class WallFollower:
    
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)


    def move_forward(self, speed):
        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0
        self.cmd_vel_pub.publish(twist_msg)

    def turn_left(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 4
        self.cmd_vel_pub.publish(twist_msg)

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = -0.9
        self.cmd_vel_pub.publish(twist_msg)

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        # """

        front_distance = msg.ranges[0]
        right_distance = min(msg.ranges[225:230])
        print(f"Front Distance: {front_distance}, Right side Distance: {right_distance}")

        if front_distance < 2:
            self.turn_left()
        
        elif right_distance > 1.9 and right_distance != float("inf"):
            self.turn_right()
        else:
            self.move_forward(0.2)


    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
