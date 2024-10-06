#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

'''
    Video Link: https://drive.google.com/file/d/1rl7oalcb9DQjwXh99eClzNmnPavioop6/view?usp=sharing
    Highlights:
        1. PD controller
        2. Lidar preprocessor
'''

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.control_signal = 0.0

        self.Kp = 0.5  # Proportional gain
        self.Kd = 1.5 # Derivative gain

        self.desired_distance = 1.0  # Desired distance from the wall (meters)
        self.prev_error_d = 0.0  # Previous distance error
        self.prev_time = rospy.Time.now().to_sec()
        self.right_distance = 0.0
        self.rate = rospy.Rate(20)
        self.tolerance = 0.05   # if error falls within this value then stop correction

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        # Assuming right-hand wall following, get the distance to the wall
        averages = self.preprocessor(msg.ranges)  
        self.right_distance = averages[23]
        # Calculate the distance error
        error_d = self.desired_distance - self.right_distance
        # Calculate time difference
        current_time = rospy.Time.now().to_sec()
        delta_time = current_time - self.prev_time

        # Calculate the derivative of the error
        derivative = (error_d - self.prev_error_d) / delta_time if delta_time > 0 else 0.0

        # PD control signal
        if abs(error_d) < self.tolerance:
            self.control_signal = 0.0
        # avoid overturning 
        else:  
            if self.control_signal > 1:
                self.control_signal = 1
            elif self.control_signal < -1:
                self.control_signal = -1
            else:
                self.control_signal = self.Kp * error_d + self.Kd * derivative

        # Update previous error and time for the next iteration
        self.prev_error_d = error_d
        self.prev_time = current_time

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        # Publish the control signal to adjust the robot's trajectory
        twist = Twist()
        while not rospy.is_shutdown():
            if self.right_distance > 1.4:
                # slow down for sharp corners
                twist.linear.x = 0.1  
                twist.angular.z = -0.1
            else:
                twist.linear.x = 0.2
                if self.control_signal and not math.isnan(self.control_signal):
                    # Steering correction based on PD controller
                    twist.angular.z = self.control_signal  
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()


    def preprocessor(self, all_ranges):
        '''
        return ranges as the average of groups of 12 values, 30 values total
        0,7,14,21  would be the index of front left back right, since lidar is counterclockwise
        '''
        ranges = [float('inf')]*30
        ranges_index = 0
        index = -6
        sum = 0
        batch = 0
        actual = 0
        for i in range(360):
            curr = all_ranges[index]
            if curr != float('inf') and curr != float('inf') and not math.isnan(curr):
                sum += curr
                actual += 1
            batch += 1
            index += 1
            if batch == 12:
                if actual != 0:
                    ranges[ranges_index] = sum/actual
                ranges_index += 1
                sum = 0
                batch = 0
                actual = 0
        return ranges

  

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
