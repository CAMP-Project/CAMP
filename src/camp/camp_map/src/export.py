#!/usr/bin/env python
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from nav_msgs.msg import OccupancyGrid

import tf2_ros as tfr

import maphelper as mh

class Camp_Map_Export:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('camp_map_export', anonymous = False)   

        in_name = rospy.get_param('map_in', 'map') 

        out_name = rospy.get_param('map_out', 'deca_map')
        
        # Subscribe to input map
        rospy.Subscriber('/'+str(in_name), OccupancyGrid, self.map_subscriber)

        # This will publish the computed map information.
        self.map_publisher = rospy.Publisher(out_name, OccupancyGrid, queue_size = 10)
        
        self.output_frame = rospy.get_param('output_frame', 'deca')

        self.tf_buffer = tfr.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tfr.TransformListener(self.tf_buffer)

        self.read_map = False

        self.next_map = OccupancyGrid()


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will grab maps as they are published.
    def map_subscriber(self, data):
        print("Data received!")
        self.read_map = True
        self.next_map = data
        print("finished the callback")


    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the merging node
    #--------------------------------------------------------------------------------------------------------------
    def main(self):
        
        # main doesn't really do anything right now.
        if self.read_map:
            print("transforming map...")
            tfd_map = mh.transform_map(self.next_map,self.output_frame,self.tf_buffer)
            if tfd_map == -1:
                print("transform failed, map not updated.")
            else:
                self.map_publisher.publish(tfd_map)
                print("published a map")
            self.read_map = False

            


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Map_Export()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(1)
