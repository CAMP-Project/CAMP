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

Edited 8/7/2021:
This script now listens to all custom messages defined within 
the "brian" package. 

---------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------------------
# Standard ROS Imports.
from geometry_msgs import msg
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
# Custom msg Imports.
from brian.msg import DecawaveLabeled, DecawaveMapLabeled, OccupancyMapLabeled, OdometryLabeled
from brian.msg import RobotKeyList
#------------------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------------------


class MM_Listener:

    # Initializer method. Defines ROS parameters (publishers, subscribers, etc.)
    def __init__(self):

        # Initialize this script as a node in ROS.
        rospy.init_node('MM_Listener', anonymous=False)

        # Variable to hold on to the list of hosts on the network. Should include itself.
        self.host_list = []
        self.current_host = None

        # Add map variables in which to store incoming data.
        self.odom_map = {} # Odom-based map storage.
        self.odom_pos = {} # Odom-based positional storage.
        self.deca_map = {} # Deca-based map storage.
        self.deca_pos = {} # Deca-based positional storage.

        # Stores subscriber objects which may be deleted later. (NOTE: Not sure if subs can be deleted.)
        self.odom_pos_subs = {} # Odom-based positional subscriber storage.
        self.odom_map_subs = {} # Odom-based map subscriber storage.
        self.deca_pos_subs = {} # Deca-based positional subscriber storage.
        self.deca_map_subs = {} # Deca-based map subscriber storage.

        # Subscribe to the list of other roscores on the current network.
        rospy.Subscriber('/host_list', RobotKeyList, self.update_list)
        rospy.sleep(1)


    # Callback to update the list of hosts on the current network.
    def update_list(self, data):
        global_keys = data.robotKeys
        
        # First, check for keys contained in the local host list that are NOT in the current global list.
        for key in self.host_list:
            if key not in global_keys:
                # Remove the entry from the local host list.
                self.host_list.pop(self.host_list.index(key))

                # Remove the entry from the odom map storage variables.
                self.odom_map.pop(key)
                self.odom_map_subs.pop(key)

                # Remove the entry from the odom pos storage variables.
                self.odom_pos.pop(key)
                self.odom_pos_subs.pop(key)

                # Remove the entry from the deca map storage variables.
                self.deca_map.pop(key)
                self.deca_map_subs.pop(key)

                # Remove the entry from the deca pos storage variables.
                self.deca_pos.pop(key)
                self.deca_pos_subs.pop(key)

        # Then, check for keys contained in the global list that are NOT in the local list.
        for key in global_keys:
            if key not in self.host_list:
                self.host_list.append(key)
                self.odom_map[key] = None
                self.odom_pos[key] = None
                self.deca_map[key] = None
                self.deca_pos[key] = None

                # Create a new odometry subscriber for the new host.
                odom_pos_label = "/" + key + "/odom_labeled"
                odom_map_label = "/" + key + "/odom_map_labeled"
                deca_pos_label = "/" + key + "/deca_labeled"
                deca_map_label = "/" + key + "/deca_map_labeled"
                self.odom_pos_subs[key] = rospy.Subscriber(odom_pos_label, OdometryLabeled, self.odom_pos_update)
                self.odom_map_subs[key] = rospy.Subscriber(odom_map_label, OccupancyMapLabeled, self.odom_map_update)
                self.deca_pos_subs[key] = rospy.Subscriber(deca_pos_label, DecawaveLabeled, self.deca_pos_update)
                self.deca_map_subs[key] = rospy.Subscriber(deca_map_label, DecawaveMapLabeled, self.deca_map_update)


    # Callback function for obtaining odometry position data.
    def odom_pos_update(self, data):
        print("The odom position is being updated")
        self.odom_pos[data.name] = data.odom


    # Callback function for obtaining odometry map data.
    def odom_map_update(self, data):
        print("The odom map is being updated")
        self.odom_map[data.name] = data.map


    # Callback function for obtaining decawave position data.
    def deca_pos_update(self, data):
        print("The deca position is being updated")
        self.deca_pos[data.name] = data.pose


    # Callback function for obtaining decawave map data.
    def deca_map_update(self, data):
        print("The deca map is being updated")
        self.deca_map[data.name] = data.map


    def main(self):

        # Method to check for None values in data maps.
        def contains_none():
            # For every key currently in the host list, check whether the storage maps have been updated
            # with non-None data. If so, return False. 
            for key in self.host_list:
                odom_pos_val = self.odom_pos[key]
                odom_map_val = self.odom_map[key]
                deca_pos_val = self.deca_pos[key]
                deca_map_val = self.deca_map[key]
                if (odom_pos_val is None) or (odom_map_val is None) or (deca_pos_val is None) or (deca_map_val is None):
                    print(type(odom_pos_val))
                    print(type(odom_map_val))
                    print(type(deca_pos_val))
                    print(type(deca_map_val))
                    return True
            
            print("There are no None values!")
            return False

        rospy.loginfo("Current hosts:")
        print(self.host_list)
        print(list(self.odom_map.keys()))
        print(list(self.odom_pos.keys()))
        print(list(self.deca_map.keys()))
        print(list(self.deca_pos.keys()))
        for key in self.host_list:
            if not contains_none():
                rospy.loginfo(str(key) + ":")
                rospy.loginfo("--Odom Pos\t: " + str(self.odom_pos[key].header.frame_id))
                rospy.loginfo("--Odom Map\t: " + str(self.odom_map[key].header.frame_id))
                rospy.loginfo("--Deca Pos\t: " + str(self.deca_pos[key].header.frame_id))
                rospy.loginfo("--Deca Map\t: " + str(self.deca_map[key].header.frame_id))

        if len(self.host_list) == 0:
            print("There don't seem to be any hosts in here!")


if __name__== '__main__':
    MML = MM_Listener()
    while not rospy.is_shutdown():
        # Run at 10 Hz.
        rospy.sleep(1)
        MML.main()