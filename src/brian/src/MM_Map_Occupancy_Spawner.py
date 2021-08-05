#!/usr/bin/env python
""""
---------------------------------------------------------------------
Author: Sean Carda
Project: CAMP
Date: 7/20/2021

Description:
This code instantiates an Occupancy grid object and an odometry
object for machines in which no lidar or motors are attached
(i.e. Virtual Machines). This is to test the service call
capabilities of the MMArbitrator when only one robot is available
for testing.

---------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------
# Standard ROS Imports.
import roslib
import math
import rospy
import tf2_py
roslib.load_manifest('rospy')
#------------------------------------------------------------------------------------
# Standard msg Imports.
from geometry_msgs.msg import PoseStamped, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, Float64
#------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------


class MM_tester:


    def __init__(self):

        # Initialize the node.
        rospy.init_node('MMA_tester', anonymous=False)

        # Instantiate two publishers for an Odometry node and OccupancyGrid node.
        self.odomPublisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.mapPublisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Create variables to hold the desired structures. 
        # First: Odometry.
        self.odom = Odometry()
        self.odom.header.frame_id = 'Turtlebot'
        self.odom.header.stamp = rospy.Time()
        self.odom.child_frame_id = 'ROS'
        self.odom.pose.pose.position.x = 2
        self.odom.pose.pose.position.y = 4
        self.odom.pose.pose.position.z = 6

        q = transformations.quaternion_from_euler(0, 0, 0)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]

        # Second: OccupancyGrid.
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'Test_Map'
        self.map.header.stamp = rospy.Time()
        self.map.info.height = 384
        self.map.info.width = 384
        self.map.info.origin.position.x = -10
        self.map.info.origin.position.y = -10


    def main(self):
        # Update the stamps of the objects.
        self.odom.header.stamp = rospy.Time()
        self.map.header.stamp = rospy.Time()

        # Publish those objects to ROS.
        self.odomPublisher.publish(self.odom)
        self.mapPublisher.publish(self.map)


if __name__== '__main__':
    tester = MM_tester()
    while not rospy.is_shutdown():
        tester.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)
