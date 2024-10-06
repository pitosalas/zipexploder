#!/usr/bin/env python3

#Video Link:
#https://drive.google.com/file/d/1DnN6ggcw0dZbYYO_3HzL_GLKxX9aZE6H/view?usp=sharing

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

DW = 1 #desired distance from wall

MAX_VEL = min(0.5*DW, 1) #max velocity in M/S
SAFE_RADIUS = 0.3 #bot should not be closer than this distance to the wall
MAX_LIDAR_DIST = 3.5 #as described by the laserscan msg.
MIN_LIDAR_DIST = 0.12
ORIENTATIONS = [0, 356, 354, 351, 342, 315, 300, 270] #wall orientations in degrees NOTE -4deg is used instead of 4.5 as recommended in paper
OBSTACLE_ORIENTATIONS = [342, 0 , 18] #orientations corresponding to a view favorable for detecting potential obstacles in front of the bot.
BASE_K = 0.3 #constant for angular velocity controller choose a value that allows desired distance to be maintained fairly closely while not over-turning.
class WallFollower:
   
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.ranges = [0]
        self.cur_orientations = ORIENTATIONS
        self.cur_obs_orientations = OBSTACLE_ORIENTATIONS
        self.Pw = [] #set of 8 wall points (x,y)
        self.Po = [] #set of 3 obstacle points (r, theta) (this is needed for the forward velocity calculation (an incoming wall is like an obstacle)) 
        self.d0 = 0 #distance ahead of robot to nearest obstacle (or wall)
        self.d = -1 #distance between robot lidar and the virtual straight wall
        self.theta = -1 #angle btwn the course angle of the bot and the direction of the virtual straight wall.
        self.y = None #representation of the virtual wall, 1st index (a) is slope, 2nd index (b) is intercept
        self.d1_prime = 0 #previous d1 value
        self.theta_prime = 0 #previous theta value
        self.v = 0 #linear velocity
        self.w = 0 #angular velocity



    #get 8 wall points from laser scanner, where origin is the base of the robot and construct a virtual straight wall using these points
    # to guide movement.
    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        self.ranges = msg.ranges

        #getting the list of valid lidar points.
        self.cur_orientations = self.cleanWallData(msg.ranges, ORIENTATIONS) 
        self.cur_obs_orientations = self.cleanObstacleData(msg.ranges, OBSTACLE_ORIENTATIONS) 

        self.Pw = [[msg.ranges[deg], self.toRadian(deg)] for deg in self.cur_orientations] 
        self.Po = [[msg.ranges[deg], self.toRadian(deg)] for deg in self.cur_obs_orientations] 
        
        #if an obstacle lidar value is above max lidar distance set it to max lidar distance
        for i in range(len(self.Po)):
            if self.Po[i][0] > MAX_LIDAR_DIST:
                self.Po[i][0] = MAX_LIDAR_DIST

        #converting Pw coords from polar to cartesian
        for i in range(len(self.Pw)):
            coord = self.Pw[i]
            r = coord[0]
            theta = coord[1]
            self.Pw[i] = self.polar_to_cartesian(r, theta) 


        self.d0 = self.avg_dist_to_obstacle(self.Po)


        #y contains formula for the virtual straight wall
        #NOTE: The coordinate plane for this line is based on the robot (robot's x and y are the corresponding x and y axes)
        self.y = self.sum_of_least_squares(self.Pw)
        
        self.d = self.distanceToLine(self.y)

        a = self.y[0] #the slope of the virtual wall
        b = self.y[1] #the y-intercept of the virtual wall

        x_intercept = (-b)/a
        self.theta = math.atan(b/x_intercept) #using arctan of x and y intercepts to get theta

        

        #calculating desired forward velocity.
        self.v = (self.d0 - SAFE_RADIUS)/(MAX_LIDAR_DIST - SAFE_RADIUS) * MAX_VEL 

        d1 = self.d - DW

        #There is likely a better way to set these constants, but I found that having a distance constant (K3)
        #that was too high led to oversteering when the desired distance was greater than 1 meter.
        K1 = BASE_K
        K2 = BASE_K
        K3 = BASE_K/DW

        #The main controller for angular velocity, if yaw (theta) is further away from the line target or distance from the 
        #target distance is higher, there will be a quicker turn response.
        change_vel = (K1*self.theta + K1*self.theta_prime) + K3*(d1 + self.d1_prime) 
       
        # uses b (essentially overall distance from the wall) to fine tune angular velocity
        self.w = 2*change_vel / b 
    
        self.theta_prime = self.theta
        self.d1_prime = d1

     

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(Twist())
        while not rospy.is_shutdown():
            curr_twist = Twist()
            #setting velocity
            curr_twist.linear.x = self.v
            curr_twist.angular.z = self.w
            self.cmd_vel_pub.publish(curr_twist)
            rate.sleep()
        
        self.cmd_vel_pub.publish(Twist())
       
    # returns average angle adjusted distance to an obstacle (or wall) ahead of the robot
    def avg_dist_to_obstacle(self, Po):
        M = len(Po)
        d0 = 0
        for i in range(M):
            d0 += self.Po[i][0]*math.cos(self.Po[i][1])
        return d0/len(self.cur_obs_orientations)

    # converts polar (r, theta) coords to cartesian (x, y)
    def polar_to_cartesian(self, r, theta):
        return [r*math.cos(theta), r*math.sin(theta)]
    
    #converts degrees to radians
    def toRadian(self, degrees):
        return degrees * (math.pi / 180)

    # returns the distance between the lidar and the virtul straight wall
    def distanceToLine(self, y):
        return abs(y[1])/(math.sqrt(y[0]**2 + 1)) 

    # ensures lidar data for wall is valid
    def cleanWallData(self, data, orientations):
        return [deg for deg in orientations if (data[deg] > MIN_LIDAR_DIST and data[deg] < MAX_LIDAR_DIST)]
    
    # ensures obstacle lidar data is valid (values above MAX_LIDAR_DIST will be set to the max value)
    def cleanObstacleData(self, data, orientations):
        return [deg for deg in orientations if (data[deg] > MIN_LIDAR_DIST)]

    #orients the robot toward the wall so it starts more smoothly.
    def find_wall(self):
        twist_angular = Twist()
        twist_forward = Twist()
        
        rate_init = rospy.Rate(1)
        rate = rospy.Rate(10)
        rate_init.sleep()


        while self.ranges[270] < MIN_LIDAR_DIST or self.ranges[270] > MAX_LIDAR_DIST:
            twist_angular.angular.z = -0.5
            self.cmd_vel_pub.publish(twist_angular)
            rate.sleep()
            
        self.cmd_vel_pub.publish(Twist())
        
    # SLS is used to construct the virtual wall based off usable lidar points
    # if less than two points are valid a line with fairly steep negative slope below the robot
    # is used to suggest that the bot is at an outside corner and should start turning
    # this is needed because the algorithm was initially programmed only for inside corners.    
    def sum_of_least_squares(self, Pw):
        a = 0
        N = len(Pw) 
        if N > 1:
        #implementation of SLS formula
            for i in range(N): 
                a += Pw[i][0]*Pw[i][1]
            sumX = 0
            sumY = 0
            sum_x_squared = 0
            for i in range(N):
                sumX += Pw[i][0]
                sumY += Pw[i][1]
                sum_x_squared += Pw[i][0] ** 2
            a -= (1/N)* sumX * sumY
            a = a / (sum_x_squared - (1/N) * (sumX**2))
            avgX = sumX / N
            avgY = sumY / N
            b = avgY - a * avgX
        else: #case for when there are not enough points for SLS
            a = -2 
            b = -1 
        return [a, b]

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().find_wall() #orients toward wall
    WallFollower().follow_wall()
