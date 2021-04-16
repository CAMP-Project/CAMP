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

# Package imports.
import tf2_ros
import roslib
import numpy;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64

class Pathfinding_Node:

    def __init__(self): 

        # Introduce tf package.
        tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)

        # Subscribe to important nodes.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)          # Subscribe to map.
        rospy.Subscriber('/odom', Odometry, self.updateRobotPosition)    # Subscribe to odometry. ** Might not be necessary.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)       # Subscribe to lidar. 

        # Create a waypoint hashmap. Stores coordinates of waypoints.
        self.waypoints = {waypoint1 : [0, 0],
                          waypoint2 : [0, 0],
                          waypoint3 : [0, 0],
                          waypoint4 : [0, 0]}


        self.botPosition = Point()  # Variable for robot position.
        self.map = OccupancyGrid()  # Variable for map storge. 
        self.lidar = LaserScan()    # Variable to access parameters of the lidar.

    # This method will update the position of the robot relative to odometry. 
    def updateRobotPosition(self, data):
        self.botPosition = data.pose.position

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.map = data

    # This method will update lidar data when new data will be available. This method grabs every parameter 
    # from the lidar node.
    def updateLidarScan(self, data):
        self.lidar = data

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

