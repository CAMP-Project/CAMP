#!/usr/bin/env python
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from nav_msgs.msg import OccupancyGrid

import tf2_ros as tfr

import maphelper as mh

class Camp_Merge:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('camp_merge', anonymous = False)   

        map1_name = rospy.get_param('local_map_name', 'map')
        
        # Subscribe to map1.
        rospy.Subscriber('/'+str(map1_name), OccupancyGrid, self.local_map_update)

        self.local_map = -1;

        # This will publish the computed map information.
        self.map_publisher = rospy.Publisher('merged_map', OccupancyGrid, queue_size = 10)

        # Variable to hold on to the list of hosts on the network. Should include itself.
        self.host_list = []
        self.current_host = None

        # Add map variables in which to store incoming data.
        self.deca_map = {} # Deca-based map storage.

        # Stores subscriber objects which may be deleted later.
        self.deca_map_subs = {} # Deca-based map subscriber storage.

        rospy.Subscriber('/host_list', RobotKeyList, self.update_list)

        self.tf_buffer = tfr.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tfr.TransformListener(self.tf_buffer)


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # Callback to update the list of hosts on the current network.
    def update_list(self, data):
        global_keys = data.robotKeys
        
        # First, check for keys contained in the local host list that are NOT in the current global list.
        for key in self.host_list:
            if key not in global_keys:
                # Remove the entry from the local host list.
                self.host_list.pop(self.host_list.index(key))

                # Remove the entry from the deca map storage variables.
                self.deca_map.pop(key)
                self.deca_map_subs.pop(key)

        # Then, check for keys contained in the global list that are NOT in the local list.
        for key in global_keys:
            if key not in self.host_list:
                self.host_list.append(key)
                self.deca_map[key] = None

                # Create a new odometry subscriber for the new host.
                deca_map_label = "/" + key + "/deca_map_labeled"
                self.deca_map_subs[key] = rospy.Subscriber(deca_map_label, DecawaveMapLabeled, self.deca_map_update)

    # This method will grab maps as they are published.
    def local_map_update(self, data):
        self.local_map = data

    # Callback function for obtaining decawave map data.
    def deca_map_update(self, data):
        #print("The deca map is being updated")
        self.deca_map[data.name] = data.map


    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the merging node
    #--------------------------------------------------------------------------------------------------------------
    def main(self):
        merged_map = self.local_map
        for new_map in self.deca_map:
            tfd_map = mh.transform_map(new_map,merged_map.header.frame_id,self.tf_buffer)
            if tfd_map != -1:
                merged_map = mh.combine_map(tfd_map,merged_map)
            else:
                rospy.INFO("transform failed, map not updated.")

        merged_map.header.stamp = rospy.Time.now()
        self.map_publisher.publish(merged_map)
        

            


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Merge()
    while not rospy.is_shutdown():
        if path.local_map is not -1:
            path.main()

        # wait for a few secs
        rospy.sleep(5)
