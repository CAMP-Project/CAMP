#!/usr/bin/env python
""""
---------------------------------------------------------------------
Author: Sean Carda
Project: CAMP
Date: 7/20/2021

Description:
This script subscribes to the Odometry and Occupancy Grid topics 
published by the MMA nodes of the cores on the current network.
Use this to verify whether or not data from other roscores is 
capable of being read.

---------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------
# Standard ROS Imports.
from geometry_msgs import msg
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
# Custom msg Imports.
from brian.msg import RobotKeyList
#------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------


class MM_Listener:

    # Initializer method. Defines ROS parameters (publishers, subscribers, etc.)
    def __init__(self):

        # Initialize this script as a node in ROS.
        rospy.init_node('MM_Listener', anonymous=False)

        # Variable to hold on to the list of hosts on the network. Should include itself.
        self.host_list = []
        self.current_host = None

        # Add a map variable in which to store Odometries.
        self.odom_map = {None: None}

        # Stores subscriber objects which may be deleted later. (NOTE: Not sure if subs can be deleted.)
        self.odom_subs = {None: None}

        # First, subscribe to the list of other hosts on the network. This is an initialization step.
        rospy.Subscriber('/host_list', RobotKeyList, self.update_list)
        rospy.sleep(1)

        # Now, iterate through the host list and create subscribers for their odometry and occupancy
        # grid nodes.
        """"
        for host in self.host_list:
            self.current_host = host
            rospy.loginfo("Attempting to create publishers for host: " + host)
            odom_sub_name = "/" + str(host) + "/odom"
            grid_sub_name = "/" + str(host) + "/odom"
            rospy.Subscriber(odom_sub_name, Odometry, self.init_odom)
        """


    def update_list(self, data):
        global_keys = data.robotKeys
        
        # First, check for keys contained in the local host list that are NOT in the current global list.
        for key in self.host_list:
            if key not in global_keys:
                self.host_list.pop(self.host_list.index(key))
                self.odom_map.pop(key)
                self.odom_subs.pop(key)

        # Then, check for keys contained in the global list that are NOT in the local list.
        for key in global_keys:
            if key not in self.host_list:
                self.host_list.append(key)
                self.odom_map[key] = None
                self.odom_subs[key] = None


    def init_odom(self, data):
        print("this is temp")


    def init_grid(self, data):
        print("this is temp")

    def main(self):
        rospy.loginfo("Current hosts:")
        for key in self.host_list:
            rospy.loginfo(str(key) + "\t: " + str(self.odom_map[key]))

        if len(self.host_list) == 0:
            print("There don't seem to be any hosts in here!")





if __name__== '__main__':
    MML = MM_Listener()
    while not rospy.is_shutdown():
        MML.main()

        # Run at 10 Hz.
        rospy.sleep(1)