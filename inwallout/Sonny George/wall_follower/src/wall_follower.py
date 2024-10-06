#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from typing import Literal, Optional


# VIDEO:
# https://drive.google.com/file/d/1iLDPVlIgrvtoXrwS28HGrkCwLz7gjTa4/view?usp=sharing


class WallFollower:
    """right-hugs the nearest wall/obstacle"""

    LIDAR_WINDOW_SMOOTHING_SIZE = 30
    RUN_LOOP_HZ = 5
    MAX_SECONDS_TO_WAIT_FOR_LIDAR = 0.8
    BASE_LINEAR_X = 0.065
    SLOW_LINEAR_X = 0.05
    MAX_ANGULAR_Z = 0.3
    MAX_INTEGRAL = 0.1
    KP = 0.08
    KD = 0.02
    KI = 0.02

    def __init__(self, target_follow_distance: float):
        self.target_follow_distance = target_follow_distance
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.on_scan_received)
        self.last_scan: Optional[LaserScan] = None
        self.rate = rospy.Rate(self.RUN_LOOP_HZ)

        self.prev_error = 0
        self.integral = 0

    def on_scan_received(self, scan):
        self.last_scan = scan

    def calculate_distance_to_nearest_rightward_wall_point(self) -> Optional[float]:
        window_size = self.LIDAR_WINDOW_SMOOTHING_SIZE
        rightward_lidar_distances = (
            self.last_scan.ranges[180 - int(window_size / 2) : 360]
            + self.last_scan.ranges[0 : int(window_size / 2)]
        )
        smoothed_distances = [
            sum(rightward_lidar_distances[i : i + window_size]) / window_size
            for i in range(180)
        ]
        smallest_distance = min(smoothed_distances)
        return smallest_distance  # , smoothed_distances.index(smallest_distance)

    def position_self_at_startup(self):
        # Get a 360 reading...

        # If there is a closest point, straight towards or away from it until D,
        # & turn until it is directly rightward

        # Else, spiral search until there is a closest point in some direction
        # and do the above
        pass

    def calculate_and_publish_follow_cmd(self, closest_rightward_wall_distance: float):
        print(
            f"target: {self.target_follow_distance:.3f} actual: {closest_rightward_wall_distance:.3f}"
        )

        error = self.target_follow_distance - closest_rightward_wall_distance

        self.integral += error / self.RUN_LOOP_HZ
        self.integral = max(min(self.integral, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)
        derivative = (error - self.prev_error) * self.RUN_LOOP_HZ

        p = self.KP * error
        i = self.KI * self.integral
        d = self.KD * derivative

        print(f"p: {p:.3f} i: {i:.3f} d: {d:.3f}")

        twist = Twist()

        # Linear x (slower if error is getting larger)
        twist.linear.x = (
            self.BASE_LINEAR_X if error > self.prev_error else self.SLOW_LINEAR_X
        )

        # Angular z
        twist.angular.z = max(min(p + i + d, self.MAX_ANGULAR_Z), -self.MAX_ANGULAR_Z)

        print(twist)
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

    def follow_wall(self):
        # Wait for lidar scans
        cycles_to_wait = max(
            int(self.RUN_LOOP_HZ / self.MAX_SECONDS_TO_WAIT_FOR_LIDAR), 1
        )
        for i in range(cycles_to_wait):
            if self.last_scan is not None:
                break
            self.rate.sleep()
        if self.last_scan is None:
            raise ValueError(
                f"No lidar scans have been received after {self.MAX_SECONDS_TO_WAIT_FOR_LIDAR:.3f} seconds."
            )

        # Move robot into a good starting location/orientation
        self.position_self_at_startup()

        # Follow wall
        while not rospy.is_shutdown():
            closest_rightward_wall_distance = (
                self.calculate_distance_to_nearest_rightward_wall_point()
            )
            if closest_rightward_wall_distance is None or math.isinf(
                closest_rightward_wall_distance
            ):
                print("EXIT CASE: No rightward wall was detected.")
                break
            self.calculate_and_publish_follow_cmd(closest_rightward_wall_distance)


if __name__ == "__main__":
    rospy.init_node("wall_follower")
    WallFollower(target_follow_distance=1.0).follow_wall()
