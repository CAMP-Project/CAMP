#!/usr/bin/env python
import math
from typing_extensions import OrderedDict
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped #Vector3, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

import tf2_ros as tfr
import tf2_geometry_msgs as tfgm

import maphelper as mh

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
        data = [-1 for i in range(0,self.map.info.width * self.map.info.height)]
        self.map.data = data

        self.tf_buffer = tfr.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tfr.TransformListener(self.tf_buffer)
        self.deca_pose = PoseStamped()
        self.maps_read = 0
        self.read_map = False


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will grab maps as they are published.
    def map_subscriber(self, data):
        print("Data received!")
        # print(max(self.map.data))
        # print(min(self.map.data))
        self.maps_read = self.maps_read + 1
        self.read_map = True
        new_map =  data
        
            
        def publish_map():
            self.map.header.stamp = rospy.Time.now()
            self.map_publisher.publish(self.map)
        
        tfd_map = mh.transform_map(new_map,self.map.header.frame_id,self.tf_buffer)
        if tfd_map != -1:
            self.map = mh.combine_map(tfd_map,self.map)
        else:
            print("transform failed, map not updated.")
            
        publish_map()
        print("finished the callback")


    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the merging node
    #--------------------------------------------------------------------------------------------------------------
    def main(self):
        
        # main doesn't really do anythin right now.
        if self.read_map:
            print("read "+str(self.maps_read)+" maps so far")
            self.read_map = False
        

            


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Merge()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.2)
