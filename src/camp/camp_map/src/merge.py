#!/usr/bin/env python
import math
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Pose, PoseStamped #Vector3, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

import tf2_ros as tfr
import tf2_geometry_msgs as tfgm

class Camp_Merge:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('camp_map', anonymous = False)   

        map1_name = rospy.get_param('map1', 'map') 

        map2_name = rospy.get_param('map2', 'map')
        
        # Subscribe to map1.
        rospy.Subscriber('/'+str(map1_name), OccupancyGrid, self.map_subscriber)

        # Subscribe to map2 if different from map1.
        if map2_name != map1_name:
            rospy.Subscriber('/'+str(map2_name), OccupancyGrid, self.map_subscriber)


        # This will publish the computed map information.
        self.map_publisher = rospy.Publisher('merged_map', OccupancyGrid, queue_size = 10)

        # Define a map used to combine all the parts of the incoming maps and to publish
        self.map = OccupancyGrid()
        self.map.header.frame_id = rospy.get_param('frame', 'map')
        self.map.header.seq  = 0
        self.map.info.resolution = 0.05
        self.map.info.width = 100
        self.map.info.height = 100
        self.map.info.origin.position.x = -self.map.info.width/2*self.map.info.resolution
        self.map.info.origin.position.y = -self.map.info.height/2*self.map.info.resolution
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        self.map.info.origin.orientation.w = 0
        data = []
        data = [50 for i in range(0,self.map.info.width * self.map.info.height)]
        self.map.data = data

        self.tf_buffer = tfr.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tfr.TransformListener(self.tf_buffer)
        self.deca_pose = PoseStamped()


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will grab maps as they are published.
    def map_subscriber(self, data):
        new_map =  data

        # This function converts a map to self.map's frame
        def transform_map(new_map):
            new_map = OccupancyGrid()
            if new_map.header.frame_id == self.map.header.frame_id:
                return new_map
            
            origin = PoseStamped()
            origin.header = new_map.header
            origin.pose = new_map.info.origin

            new_origin = PoseStamped()

            try:
                transform = self.tf_buffer.lookup_transform(self.map.header.frame_id,new_map.header.frame_id,rospy.Time(0),rospy.Duration(1.0))
                new_origin = tfgm.do_transform_pose(origin, transform)
            except Exception as x:
                rospy.logwarn("transform goofed:\n%s",str(x))
                # raise RuntimeError("Transform not found")
                return OccupancyGrid()

            tfd_map = new_map
            tfd_map.info.origin = new_origin
            tfd_map.header.frame_id = self.map.header.frame_id

            # TODO: take the map data and rotate it in a way where the info.origin orientation matches the output map.
            # This will likely include moving the origin and expanding the size of the map

            return tfd_map

        # This function combines the new map's data with the existing map.
        def combine_map(new_map):
            # first, make sure we have an output map big enough to hold all the data based off of self.map
            # make sure to set the map.info to match

            # next, populate the output map based on given instructions. example might be to keep all data that is 100% certain, and ignore -1's or 50's

            # sew up the output map so it's ready to be saved

            return out_map
        
        tfd_map = transform_map(new_map)
        out_map = combine_map(tfd_map)
        self.map = out_map
        # map publishign is handled in main.


    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the merging node
    #--------------------------------------------------------------------------------------------------------------
    def main(self):
        # This function publishes the combined map
        def publish_map():
            self.map.header.stamp = rospy.Time.now()
            self.map_publisher.publish(self.map)

        ### REAL MAIN ###
        publish_map()


            


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Merge()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.2)
