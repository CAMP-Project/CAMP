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

Edited 8/7/2021:
This code now instantiates several custom topics which will be used
to verify communication capabilities for the MMA (without service
calls).

---------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------------------
# Standard ROS Imports.
import roslib
import math
import rospy
import tf2_py
roslib.load_manifest('rospy')
#------------------------------------------------------------------------------------------------
# Standard msg Imports.
from geometry_msgs.msg import PoseStamped, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, Float64
#------------------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------------------
# Custom Message Imports.
from brian.msg import DecawaveLabeled, DecawaveMapLabeled, OccupancyMapLabeled, OdometryLabeled
#------------------------------------------------------------------------------------------------


class MM_tester:


    def __init__(self):

        # Initialize the node.
        rospy.init_node('MMA_tester', anonymous=False)

        # Instantiate two publishers for an Odometry node and OccupancyGrid node.
        self.odomPublisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomMapPublisher = rospy.Publisher('odom_map', OccupancyGrid, queue_size=10)
        self.decaPublisher = rospy.Publisher('filtered', Twist, queue_size=10)
        self.decaMapPublisher = rospy.Publisher('deca_map', OccupancyGrid, queue_size=10)

        rospy.loginfo("Preparing topics.")

        # Create variables to hold the desired structures. 
        """"
        --------------------------
        First: Odometry.
        --------------------------
        """
        rospy.loginfo("Generating odometry...")
        self.odom = Odometry()
        self.odom.header.frame_id = 'Turtlebot_Odom'
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

        """"
        --------------------------
        Second: Odom-based Map.
        --------------------------
        """
        rospy.loginfo("Generating map...")
        self.odom_map = OccupancyGrid()
        self.odom_map.header.frame_id = 'Odom_Map'
        self.odom_map.header.stamp = rospy.Time()
        self.odom_map.info.height = 384
        self.odom_map.info.width = 384
        self.odom_map.info.origin.position.x = -10
        self.odom_map.info.origin.position.y = -10

        """"
        --------------------------
        Third: Decawave Position.
        --------------------------
        """
        rospy.loginfo("Generating decawave...")
        self.deca = Twist()
        self.deca.linear.x = 2
        self.deca.linear.y = 4
        self.deca.linear.z = 0

        """"
        --------------------------
        Third: Deca-based Map.
        --------------------------
        """
        rospy.loginfo("Generating decawave map...")
        self.deca_map = OccupancyGrid()
        self.deca_map.header.frame_id = 'Deca_Map'
        self.deca_map.header.stamp = rospy.Time()
        self.deca_map.info.height = 384
        self.deca_map.info.width = 384
        self.deca_map.info.origin.position.x = -10
        self.deca_map.info.origin.position.y = -10


    def main(self):
        # Update the stamps of the objects.
        self.odom.header.stamp = rospy.Time()
        self.odom_map.header.stamp = rospy.Time()
        self.deca_map.header.stamp = rospy.Time()

        # Publish those objects to ROS.
        self.odomPublisher.publish(self.odom)
        self.odomMapPublisher.publish(self.odom_map)
        self.decaMapPublisher.publish(self.deca_map)
        self.decaPublisher.publish(self.deca)


if __name__== '__main__':
    tester = MM_tester()
    while not rospy.is_shutdown():
        tester.main()

        # Run at 10 Hz.
        rospy.sleep(1)
