#!/usr/local/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
from sklearn.linear_model import LinearRegression

# Constants
MAX_RANGE = 1


class LidarReader:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.right_range_pub = rospy.Publisher(
            "/wall_data", Point, queue_size=1)
        self.ranges = None
        self.right_ranges = None
        self.rate = rospy.Rate(20)

        # List of lidar angles we want to grab
        self.target_right_ranges = [0, -4, -6, -9, -18, -45, -60, -90]
        self.np_target_right_ranges = np.array(self.target_right_ranges)
        # self.target_right_ranges = [-30, -60, -90, -120, -150]

    def scan_cb(self, msg):
        '''
            Scan callback function
        '''
        max_range = min(MAX_RANGE, msg.range_max)
        # Filter out data
        self.right_ranges = [msg.ranges[i]
                             if ((msg.ranges[i] < max_range)
                                 and (msg.ranges[i] > msg.range_min)) else -1
                             for i in
                             self.target_right_ranges]
        np_right_ranges = np.array(self.right_ranges)

        # Pull robot's relative distance and angle to the wall
        angles, distance = self.detect_wall(
            self.np_target_right_ranges, np_right_ranges)

        # Create a point to send our data
        point = Point()
        point.x = angles if angles is not None else 0
        point.y = distance if distance is not None else 0
        point.z = max_range
        self.right_range_pub.publish(point)

    def detect_wall(self, angles, distances):
        '''
            Detects the robot's relative distance and angle to the wall
        '''
        valid_indices = distances > 0

        filtered_angles = angles[valid_indices]
        filtered_distances = distances[valid_indices]

        # Convert polar coordinates to Cartesian
        x = filtered_distances * np.cos(np.radians(filtered_angles))
        y = filtered_distances * np.sin(np.radians(filtered_angles))

        if len(filtered_distances) < 2:
            return None, None  # Not enough data for wall detection

        # Reshape for sklearn
        X = x.reshape(-1, 1)
        Y = y.reshape(-1, 1)

        # Fit line
        reg = LinearRegression().fit(X, Y)

        # Calculate wall angle
        wall_slope = reg.coef_[0][0]
        wall_angle = np.degrees(np.arctan(wall_slope))

        # Calculate perpendicular distance
        intercept = reg.intercept_[0]
        distance = abs(intercept) / np.sqrt(1 + wall_slope**2)

        return wall_angle, distance


def run(self):
    # For debugging
    while not rospy.is_shutdown():
        if self.right_ranges is not None:
            print(len(self.right_ranges))


if __name__ == "__main__":
    rospy.init_node("wall_detector")
    lidar_reader = LidarReader()
    rospy.spin()
