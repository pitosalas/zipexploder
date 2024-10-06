#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#The follwing link is for video submission
#https://drive.google.com/file/d/1fq3A0xH03hPg6FmI1D56vc_I7JxWGnav/view?usp=sharing

class WallFollower:
    def __init__(self):

        self.ranges = []
        self.angle_increment = 0
        self.PID = [0.4, 0.01, 0.8, 0.05] 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        self.linear_speed = 0.5
        self.distance_margin = 1.0
        self.rate = rospy.Rate(1/self.PID[3])

    def scan_cb(self, msg):
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment

    def runWithAdjust(self):
        rospy.loginfo('runWithAdjust')
        twist = Twist()
        prev_error = 0
        sum_error = 0

        while not rospy.is_shutdown():
            dist = float('inf')

            rospy.wait_for_message('/scan', LaserScan)  

            if self.ranges[0] < 1.1:
                rospy.loginfo('inner-corner')
                init_time = rospy.Time.now()
                turned_dist = 0
                index = self.ranges.index(min(self.ranges))
                while index <85 or index >95 :
                    twist.angular.z = -0.5
                    twist.linear.x = 0
                    self.cmd_vel_pub.publish(twist)
                    index = self.ranges.index(min(self.ranges))
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)

            
            if all(i == float('inf') for i in self.ranges[0:85]):
                rospy.loginfo('out-corner')
                init_time = rospy.Time.now()
                turned_dist = 0
                while turned_dist < math.pi / 2:
                    twist.angular.z = 0.5
                    twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(twist)
                    turned_dist = (rospy.Time.now() - init_time).to_sec() * 0.5
            


            for i in range(45, 135): 
                if self.ranges[i] < dist:
                    dist = self.ranges[i]

            curr_error = dist - self.distance_margin
            sum_error += curr_error * self.PID[3]
            PID_cal = self.PID[0] * curr_error + self.PID[2] * (curr_error - prev_error) / self.PID[3] + self.PID[1] * sum_error
            prev_error = curr_error

            twist.angular.z = PID_cal
            twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def adjustAngle(self):
        rospy.loginfo('adjustAngle')
        rospy.wait_for_message('/scan', LaserScan)
        twist = Twist()
        init_time = rospy.Time.now()
        turned_angle = 0
        index = self.ranges.index(min(self.ranges))
        while index <75 or index >105 :
            twist.angular.z = -0.2
            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)
            index = self.ranges.index(min(self.ranges))
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        self.adjustAngle()
        self.runWithAdjust()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()