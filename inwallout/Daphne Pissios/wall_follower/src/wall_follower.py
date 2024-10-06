#!/usr/bin/env python3
#https://drive.google.com/file/d/1aKpHjO-xXjGU0_DqHw6UDQZ0X6IBjdLB/view?usp=sharing
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# from pid import PID

MOVE_SPEED = 0.1
TURN_SPEED = 0.2

class PID:
    def __init__(self, Kp, Ki, Kd, distance = 1):
        self.Kp = Kp #proportional gain
        self.Ki = Ki #inregral gain
        self.Kd = Kd #derivative gain

        self.distance = distance
        self.d_error = 0 #derivative error
        self.i_error = 0 #integral error

    def update(self, cur_distance):
        # while not rospy.is_shutdown():
        error = self.distance - cur_distance
        P = self.Kp * error 

        self.i_error += error 
        I = self.Ki * self.i_error 

        derivative = (error - self.d_error) 
        D = self.Kd * derivative

        self.d_error = error

        print(f"{P:0.3f},{I:0.3f},{D:0.3f}")
        return P + I + D

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        self.twist = Twist()
        self.wall_distance = 1

        self.pid_values = PID(Kp = 1.0, Ki = 0.0, Kd = 0.1, distance = self.wall_distance)
        self.current_wall_distance = 0.0

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        # while not rospy.is_shutdown():
        wall_distance = min(msg.ranges[0:30] + msg.ranges[330:360])  # scans left wall

        signal = self.pid_values.update(wall_distance) #use pid to get direction

        # if signal is None or math.isfinite(signal): 
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        # else: 
        self.twist.linear.x = MOVE_SPEED
        self.twist.angular.z = -signal  # positive turns right, negative turns left

        self.cmd_vel_pub.publish(self.twist)


    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
