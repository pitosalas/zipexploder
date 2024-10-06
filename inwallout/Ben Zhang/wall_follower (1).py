#!/usr/bin/env python3
# https://drive.google.com/file/d/1INx4DD1sXj1xyhwo72-hoTB_8ZNSPFs9/view?usp=sharing 
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import time

class WallFollower:
    def __init__(self):
        # Publisher and Subscriber initialization
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_ = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
        # Wall following variables
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            0: 'Find the wall',
            1: 'Turn right',
            2: 'Follow the wall',
        }

        # PID controller variables
        self.prev_error = 0.0
        self.prev_time = time.time()

        # Parameters
        self.desired_distance = 1.2  # Desired distance to the wall
        self.kp = 0.6                # Proportional gain
        self.ki = 0.0                # Integral gain(Unused)
        self.kd = 0.5                # Derivative gain
        self.max_angular_speed = 1.0
        self.min_angular_speed = -1.0

        # Laser scan data variables
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.ranges = None
        self.num_ranges = None

        # Left distance history for smoothing
        self.left_distance_history = []

    def clbk_laser(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.num_ranges = len(self.ranges)

        # Function to convert angle to index
        def angle_to_index(angle):
            index = int((angle - self.angle_min) / self.angle_increment)
            index = max(0, min(index, self.num_ranges - 1))
            return index

        # Compute indices for regions
        self.regions_ = {
            'right':  min([self.ranges[i] if not math.isinf(self.ranges[i]) else 10
                           for i in range(angle_to_index(-math.pi/2), angle_to_index(-math.pi/4))] or [10]),
            'fright': min([self.ranges[i] if not math.isinf(self.ranges[i]) else 10
                           for i in range(angle_to_index(-math.pi/4), angle_to_index(-math.pi/8))] or [10]),
            'front':  min([self.ranges[i] if not math.isinf(self.ranges[i]) else 10
                           for i in range(angle_to_index(-math.pi/8), angle_to_index(math.pi/8))] or [10]),
            'fleft':  min([self.ranges[i] if not math.isinf(self.ranges[i]) else 10
                           for i in range(angle_to_index(math.pi/8), angle_to_index(math.pi/4))] or [10]),
            'left':   min([self.ranges[i] if not math.isinf(self.ranges[i]) else 10
                           for i in range(angle_to_index(math.pi/4), angle_to_index(math.pi/2))] or [10]),
        }

        self.take_action()

    def change_state(self, state):
        if state != self.state_:
            rospy.loginfo('Wall follower - [%s] - %s', state, self.state_dict_[state])
            self.state_ = state

    def take_action(self):
        d = self.desired_distance  # Desired distance to the wall

        rospy.loginfo("Regions: %s", self.regions_)

        if self.regions_['front'] > d and self.regions_['fleft'] > d and self.regions_['left'] > d:
            self.change_state(0)  # Find the wall
        elif self.regions_['front'] < d:
            self.change_state(1)  # Turn right
        elif self.regions_['left'] < 1.5 * d:
            self.change_state(2)  # Follow the wall
        else:
            self.change_state(0)  # Find the wall

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0  # Move straight to find the wall
        return msg

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.7  # Turn right to avoid obstacle in front
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.22

        # Filter the left distance using a moving average
        left_distances = self.get_left_distances()
        if left_distances:
            current_left_distance = sum(left_distances) / len(left_distances)
        else:
            current_left_distance = 10  # Default value if no valid readings

        # Update history (keep the last N values)
        N = 20
        self.left_distance_history.append(current_left_distance)
        if len(self.left_distance_history) > N:
            self.left_distance_history.pop(0)

        # Compute smoothed left_distance
        left_distance = sum(self.left_distance_history) / len(self.left_distance_history)

        # PID Controller
        error = self.desired_distance - left_distance
        current_time = time.time()
        delta_time = current_time - self.prev_time if self.prev_time else 0.1  # Default delta time
        self.prev_time = current_time

        # Derivative calculation
        derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0.0
        self.prev_error = error

        # Compute control output
        control = self.kp * error + self.kd * derivative

        # Adjust angular speed
        msg.angular.z = control
        # Limit angular speed
        msg.angular.z = max(self.min_angular_speed, min(self.max_angular_speed, msg.angular.z))

        rospy.loginfo("Follow wall - Left distance: %.2f, Error: %.2f, Control: %.2f",
                      left_distance, error, control)

        return msg

    def get_left_distances(self):
        # Get distances in the left region (from 30 degrees to 90 degrees)
        left_angles = [math.radians(angle) for angle in range(30, 91, 5)]
        left_indices = [int((angle - self.angle_min) / self.angle_increment) for angle in left_angles]
        left_indices = [max(0, min(index, self.num_ranges - 1)) for index in left_indices]

        left_distances = []
        for i in left_indices:
            distance = self.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance):
                left_distances.append(distance)

        return left_distances

    def main(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = Twist()
            if self.state_ == 0:
                msg = self.find_wall()
            elif self.state_ == 1:
                msg = self.turn_right()
            elif self.state_ == 2:
                msg = self.follow_the_wall()
            else:
                rospy.logerr('Unknown state!')
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            self.pub_.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    wall_follower.main()
