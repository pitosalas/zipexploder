#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

#Lidar detector range from turtlebot3
MAX_RANGE = 3.5
MIN_RANGE = 0.1

#Speed limitation
MAX_SPEED = 0.5
ADJUST_SPEED = 0.3

STD_DIST = 1.0 #the distance between robot and wall

#This robot would follow wall from its right-hand side
class WallFollower:

    def __init__(self):
        self.state = 0
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        self.indicator = False

        self.twist = Twist()
        self.cur_value = self.front = self.left = self.exact_left = self.right = None

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        #Data holder
        self.cur_value = msg.ranges

        #DATAs from all direction
        self.front = min(min(self.cur_value[:20] + self.cur_value[-20:]), MAX_RANGE)
        self.back = min(min(self.cur_value[160:200]), MAX_RANGE)

        self.right = min(min(self.cur_value [60:110]), MAX_RANGE)
        self.left = min(min(self.cur_value [250:290]), MAX_RANGE)

        #Front-left and Front-right
        self.fl = min(min(self.cur_value[21:59]), MAX_RANGE)
        self.fr = min(min(self.cur_value[291:349]), MAX_RANGE)

        self.cb_interpreter_with_wrong_direction_fix()

    def cb_interpreter_with_wrong_direction_fix(self):

        #First rotate its direction to wall direction
        if self.indicator == False:
            #rotate the robot to the wall direction first
            if self.fr >= STD_DIST:
                self.state = 1
            #only do that once
            else:
                self.indicator = True

        #Return to normal condition after the direction is fixed
        else:
            #back only
            if self.front > STD_DIST and self.fr > STD_DIST and self.fl > STD_DIST and self.right > STD_DIST and self.back < STD_DIST:
                self.state = 1
            #nothing around
            elif self.front > STD_DIST and self.fr > STD_DIST and self.fl > STD_DIST and self.right > STD_DIST:
                self.state = 0
            #right only, this is for outer corner
            elif self.front > STD_DIST and self.fr > STD_DIST and self.fl > STD_DIST and self.right < STD_DIST:
                self.state = 2
            #front only
            elif self.front < STD_DIST and self.fr > STD_DIST and self.fl > STD_DIST:
                self.state = 1
            #fr and right
            elif self.front > STD_DIST and self.fr > STD_DIST and self.fl < STD_DIST and self.right < STD_DIST:
                self.state = 2
            #fl only
            elif self.front > STD_DIST and self.fr < STD_DIST and self.fl > STD_DIST:
                self.state = 0
            #front and fr
            elif self.front < STD_DIST and self.fr > STD_DIST and self.fl < STD_DIST:
                self.state = 1
            #front and fl
            elif self.front < STD_DIST and self.fr < STD_DIST and self.fl > STD_DIST:
                self.state = 1
            #front and fr and fl
            elif self.front < STD_DIST and self.fr < STD_DIST and self.fl < STD_DIST:
                self.state = 1
            #fr and fl
            elif self.front > STD_DIST and self.fr < STD_DIST and self.fl < STD_DIST:
                self.state = 0

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """

        # Known flaw: The robot will "Shake". It is because the angle selection of the Ridar could be improved.

        self.twist =  Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cur_value != None:
                if self.state == 0:
                    #Look for the wall
                    self.twist.linear.x = ADJUST_SPEED
                    self.twist.angular.z = -ADJUST_SPEED
                elif self.state == 1:
                    #Rotate left
                    self.twist.angular.z = MAX_SPEED
                elif self.state == 2:
                    #Move forward
                    self.twist.linear.x = MAX_SPEED
                    pass
                
                self.cmd_vel_pub.publish(self.twist)
                rate.sleep()

        #raise NotImplementedError


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()
    #keep the program looping
    rospy.spin()
