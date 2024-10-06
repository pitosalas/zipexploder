#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MyOdom:
    def __init__(self):
        self.old_pose = None  
        self.dist = 0.0
        self.yaw = 0.0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('/my_odom', Point, queue_size=1)
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose

        #initiate the old_pose, since leave it none will cause error to furthur function
        if (self.old_pose == None):
            self.old_pose = cur_pose

        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        #Calcluate the distance travelled
        x1 = cur_pose.position.x
        x2 = self.old_pose.position.x
        y1 = cur_pose.position.y
        y2 = self.old_pose.position.y
        dist_hold = math.hypot(x1 - x2, y1 - y2)

        #I followed the given instruction website first, so I didn't use the "f" to cut the float
        if (dist_hold >= 0.00001):
            self.dist += dist_hold
        self.old_pose = cur_pose
        #raise NotImplementedError

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """

        orientation_hold = cur_orientation
        quat_hold = (orientation_hold.x, orientation_hold.y, orientation_hold.z, orientation_hold.w)
        (roll,pitch,yaw) = euler_from_quaternion(quat_hold)

        #self.yaw is going to be publish to /my_odom
        #yaw is calculated from cur_orientation
        yaw_hold = abs(self.yaw - yaw)

        #still, I use the first website instead of "f" to cut the float
        if (yaw_hold >= 0.02):
            self.yaw = yaw
        #raise NotImplementedError

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.

        self.cur_data = Point()
        self.cur_data.x = self.dist
        self.cur_data.y = self.yaw
        self.cur_data.z = 0.0

        #This is a helper function for me to check how the program going
        rospy.loginfo(f"Distance: {self.cur_data.x}, Yaw: {self.cur_data.y}")
        self.my_odom_pub.publish(self.cur_data)

        #raise NotImplementedError
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
