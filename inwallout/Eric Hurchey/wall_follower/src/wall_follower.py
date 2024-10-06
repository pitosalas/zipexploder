#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Video: https://drive.google.com/file/d/19eTjncTKOWnChAJY-A4aqMNi7H9aE8w9/view?usp=sharing

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.data = None
        self.wall_distance = 1.0
        self.state = 0 
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()   
        self.Kp_angular = 1.0  # Adjust this gain if the robot is too erratic when turning
        self.Kd_angular = 0.3  # Angular velocity derivative gain to smooth out turns
        self.max_linear_speed = 0.4  # Reduced max speed for better control
        self.Kp_linear = 0.2  # Proportional gain for distance correction
        self.rotate_speed = -0.5  # Negative for right rotation (use this to turn)
        
        rospy.loginfo("Wall Follower Initialized")

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        self.data = msg

    def averager(self, ranges):
        real = 0
        sum = 0
        for i in ranges:
            if i != float('inf') and not math.isnan(i):
                real += 1
                sum += i
        return float('inf') if real == 0 else sum/real

    def cardinal_directions(self, ranges):
        directions = {"Front": float('inf'), "Back": float('inf'), "Left": float('inf'), "Right": float('inf')}
        plus_minus = 9
        Front = ranges[-plus_minus:] + ranges[:plus_minus]
        Back = ranges[180 - plus_minus:180+plus_minus]
        Left = ranges[90 - plus_minus:90+plus_minus]
        Right = ranges[270-plus_minus:270 + plus_minus]
        directions_before_processed = {"Front": Front, "Back": Back, "Left": Left, "Right": Right}
        for direction, data in directions_before_processed.items():
            directions[direction] = self.averager(data)
        return directions

    def preprocessor(self, all_ranges):
        ranges = [float[('inf')]] * 20
        ranges_index = 0
        index = -9
        sum = 0
        batch = 0
        actual = 0
        for i in range(360):
            curr = all_ranges[index]
            if curr != float('inf') and not math.isnan(curr):
                sum += curr
                actual += 1
            batch += 1
            index += 1
            if batch == 18:
                if actual != 0:
                    ranges[ranges_index] = sum/actual
                ranges_index += 1
                sum = 0
                batch = 0
                actual = 0
        return ranges

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        wall_distance = self.wall_distance 
        while not rospy.is_shutdown():
            if self.data:
                ranges = self.data.ranges
                # Replace 0.0 readings with inf to ignore invalid measurements
                ranges = [r if r != 0.0 else float('inf') for r in ranges]
                directions = self.cardinal_directions(ranges)
                left_front = directions['Left']
                front_distance = directions['Front']
                min_range = min(ranges)
                min_index = ranges.index(min_range)
                angle_to_closest = min_index
                angle_to_closest_rad = self.data.angle_min + min_index * self.data.angle_increment

                if angle_to_closest_rad > math.pi:
                    angle_to_closest_rad -= 2 * math.pi
                elif angle_to_closest_rad < -math.pi:
                    angle_to_closest_rad += 2 * math.pi
                current_time = rospy.Time.now()
                delta_time = (current_time - self.last_time).to_sec()
                if delta_time == 0:
                    delta_time = 0.1
                self.last_time = current_time

                if self.state == 0:
                    # State 0: Approach the wall
                    if min_range > self.wall_distance:
                        distance_error = min_range - self.wall_distance
                        self.twist.linear.x = min(self.Kp_linear * distance_error, self.max_linear_speed)
                        self.twist.angular.z = self.Kp_angular * angle_to_closest_rad
                    else:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        initial_angle = angle_to_closest_rad
                        self.state = 1  # Transition to State 1
                        rospy.loginfo("Reached wall. Transitioning to State 1: Rotating 90 degrees to the right.")
                elif self.state == 1:
                    # State 1: Rotate 90 degrees to the right
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.rotate_speed  # Rotate right

                    # Update the angle to the closest point
                    min_range = min(ranges)
                    min_index = ranges.index(min_range)
                    current_angle = self.data.angle_min + min_index * self.data.angle_increment
                    current_angle = (current_angle + math.pi) % (2 * math.pi) - math.pi

                    # Calculate angle turned
                    angle_diff = (initial_angle - current_angle) % (2 * math.pi)
                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi  # Normalize angle_diff to [-pi, pi]

                    angle_turned = abs(angle_diff)

                    rospy.loginfo(f"Angle turned: {math.degrees(angle_turned):.2f} degrees")

                    # Check if the robot has turned approximately 90 degrees
                    if angle_turned >= math.radians(88):
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.0
                        self.state = 2
                        rospy.loginfo("Rotation complete. Robot has turned approximately 90 degrees to the right.")
                    else:
                        rospy.loginfo("Rotating right...")
                elif self.state == 2:
                    #Robot should then go straight, parallel to the wall while also keeping its distance of 1 meter
                    # Control angular velocity to stay parallel and linear velocity to maintain distance
                    error_angle = left_front - self.wall_distance  # Difference between desired distance and current distance

                    # Calculate angular velocity using PD control (no integral term)
                    derivative = (error_angle - self.previous_error) / delta_time
                    angular_correction = self.Kp_angular * error_angle + self.Kd_angular * derivative
                    self.previous_error = error_angle

                    # Set linear velocity and apply angular correction
                    self.twist.linear.x = self.max_linear_speed
                    self.twist.angular.z = angular_correction

                    # Detect if the robot approaches a corner (wall disappears)
                    if front_distance < wall_distance + 0.3:  # There's a wall ahead
                        rospy.loginfo("Wall ahead. Transitioning to State 1: Turning.")
                        self.twist.linear.x = 0.0
                        self.state = 1  # Go back to turning

                    elif left_front > wall_distance + 0.5:  # Wall on the left disappears
                        rospy.loginfo("Corner detected. Stopping the robot.")
                        self.twist.linear.x = 0.0
                        self.state = 1  # Turn to follow the next wall
                else:
                    # Default case (should not occur)
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
