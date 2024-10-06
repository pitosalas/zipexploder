#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# The video: https://drive.google.com/file/d/1xsJanNRlzyfEuEwvfbVSxiGw2sYyhbox/view?usp=sharing

class WallFollower:
    def __init__(self):
        # Publisher to control the robot's velocity using the 'cmd_vel' topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Subscriber to get LIDAR scan data from the '/scan' topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        # Desired distance from the wall (in meters)
        self.desired_distance = 0.5
        
        # Rate at which control commands will be sent (10 Hz)
        self.rate = rospy.Rate(10)
        
        # PID controller parameters
        self.kp = 1.0  # Proportional gain (P)
        self.ki = 0.0  # Integral gain (I)
        self.kd = 0.5  # Derivative gain (D)
        
        # Variables to store the error terms for the PID controller
        self.prev_error = 0.0  # Previous error for calculating the derivative
        self.integral = 0.0  # Accumulated error for the integral term

        # Print statement for logging
        print(f"Wall Follower initialized. Desired distance from the wall: {self.desired_distance}")

    def scan_cb(self, msg):
        """
        Callback function to process LIDAR scan data.
        Extracts the front and right distance readings from the scan data and
        calls the wall-following function.
        """
        # Calculate front distance (average over a small range of angles at the front)
        front_distance = min(msg.ranges[0:30] + msg.ranges[-30:])  # Indices cover front of the robot
        
        # Calculate right distance (average over a small range of angles at the right)
        right_distance = min(msg.ranges[90:120])  # Indices cover right side of the robot
        
        # Debugging output: print the front and right distances
        print(f"Front Distance: {math.degrees(front_distance):.2f} meters and Right Distance: {math.degrees(right_distance):.2f}")
        
        # Follow the wall using the front and right distances
        self.follow_wall(front_distance, right_distance)

    def follow_wall(self, front_distance, right_distance):
        """
        Controls the robot's motion to follow the wall using LIDAR data and a PID controller.
        Adjusts the robot's angular velocity to maintain a desired distance from the wall.
        """
        # Create a Twist message to store linear and angular velocities
        twist = Twist()
        
        # If the robot doesn't detect a wall on the right (infinite or zero distance)
        if right_distance == math.inf or right_distance == 0:
            # Debugging output: print the logging information
            print("No wall detected on the right, turning to find the wall...")
            twist.linear.x = 0  # Stop forward motion
            twist.angular.z = -1.0  # Turn in place to find the wall
        else:
            # PID control for angular velocity
            error = right_distance - self.desired_distance  # Difference between actual and desired distance
            self.integral += error  # Accumulate the error over time
            derivative = error - self.prev_error  # Rate of change of the error
            
            # PID controller formula: proportional + integral + derivative
            angular_v = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Print the logging information
            print(f"Error: {error:.2f}, Integral: {self.integral:.2f}, Derivative: {derivative:.2f}")
            print(f"Calculated angular velocity: {angular_v:.2f}")
            
            # If there's an obstacle in front, adjust angular velocity to avoid it
            if front_distance != math.inf and front_distance < 1.0:
                angular_v += -0.5 / front_distance  # Increase turn rate based on proximity
                print(f"Obstacle detected in front! Adjusting angular velocity: {angular_v:.2f}")
            
            # Set a constant forward speed
            twist.linear.x = 0.2
            
            # Apply the computed angular velocity (adjust turning based on wall distance)
            twist.angular.z = angular_v

            # Print current motion commands
            print(f"Moving forward with linear velocity: {twist.linear.x} and angular velocity: {twist.angular.z:.2f}")
            
            # Update the previous error for the next iteration
            self.prev_error = error

        # Publish the velocity command to move the robot
        self.cmd_vel_pub.publish(twist)
        
        # Sleep to maintain the loop rate (10 Hz)
        self.rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node named 'wall_follower'
    rospy.init_node('wall_follower')

    # Print a message indicating that the node has started
    print("Wall Follower node has started.")
    
    # Create a WallFollower object to start the process
    WallFollower()
    
    # Keep the node running and responsive to the LIDAR data
    rospy.spin()