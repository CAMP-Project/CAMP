#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 4/15/2021
# Description     : This code is adapted from an algorithm developed by Dr. Michael 
#                   McCourt. This code uses LiDAR and probabilistic map data to 
#                   generate waypoints for robotic pathfinding. 
#
#
#
# Change Log:
#
# 4/15/2021 - Sean Carda: Created script and initial methods.
#
#
#
#----------------------------------------------------------------------------------------


# FOR DEVELOPMENT
# ---------------
# This code will subscribe to multiple Turtlebot topics such as its odometry and map.
# This code will use odometry to localize itself within the map. It will use map
# data to determine where to scan and where to generate waypoints.
# The current functionality outline of this code is as follows (this is subject to change):
# 1. Initialize a set of waypoints in one direction.
# 2. Check lidar distances.
# 3a. If no objects are detected to impede movement, travel to waypoint, shift waypoints, and create new waypoint.
# 3b. If an object is detected, reset the waypoint list. 

import roslib
roslib.load_manifest('rospy')
import numpy;
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64

class Pathfinding_Node:

    def __init__(self): 
        rospy.Subscriber('/dwm1001/tag1', Tag, self.tag_update)
        rospy.Subscriber('/odom', Odometry, self.velocity_update)


        self.waypoints = {waypoint1 : [0, 0],
                        waypoint2 : [0, 0],
                        waypoint3 : [0, 0],
                        waypoint4 : [0, 0]}

        self.botPosition = Point()

    # This method will update the position of the robot relative to odometry. 
    def updateRobotPosition(self, data):

    # This method will read in LiDAR data to determine if the current path needs to be reset.
    def readLidar():

    # This method will reset the waypoint list.
    def resetWaypoints():
    
    # This method will create a new waypoint once the robot is within a certain distance
    # to waypoint 1.
    def createNewWaypoint():


if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        print("Something should happen here!")

