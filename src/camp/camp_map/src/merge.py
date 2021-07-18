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

        # This function converts a map to self.map's frame
        def transform_map(new_map):
            # Declaration to allow tab-completion 
            #new_map = OccupancyGrid()

            # TODO: account for the possibility of different resolutions?

            if new_map.header.frame_id == self.map.header.frame_id:
                print("bad frame")
                return new_map

            #! Assuming same frame for now

            # origin = PoseStamped()
            # origin.header = new_map.header
            # origin.pose = new_map.info.origin

            # new_origin = PoseStamped()

            # try:
            #     transform = self.tf_buffer.lookup_transform(self.map.header.frame_id,new_map.header.frame_id,rospy.Time(0),rospy.Duration(1.0))
            #     new_origin = tfgm.do_transform_pose(origin, transform)
            # except Exception as x:
            #     rospy.logwarn("transform goofed:\n%s",str(x))
            #     # raise RuntimeError("Transform not found")
            #     return OccupancyGrid()

            # tfd_map = new_map
            # tfd_map.info.origin = new_origin
            # tfd_map.header.frame_id = self.map.header.frame_id

            # # TODO: take the map data and rotate it in a way where the info.origin orientation matches the output map.
            # # This will likely include moving the origin and expanding the size of the map

            # return tfd_map

            print("Frames don't match!")

        # This function combines the new map's data with the existing map.
        def combine_map(new_map):
            # Declaration to allow tab-completion 
            #new_map = OccupancyGrid()
            #! Assuming rotation is taken care of in 
            # first, make sure we have an output map big enough to hold all the data based off of self.map
            # make sure to set the map.info to match

            # copy the data from the existing saved map into a new map
            out_map = OccupancyGrid()
            out_map.header = self.map.header
            out_map.info.resolution = self.map.info.resolution

            # check to see if the new map has an origin further to the bottom corner than the saved one.
            # assign a new origin and width/height
            meter_difference = self.map.info.origin.position.x - new_map.info.origin.position.x
            pixel_difference = int(round(meter_difference / self.map.info.resolution))

            out_map.info.width = max(new_map.info.width - pixel_difference,self.map.info.width + pixel_difference,new_map.info.width,self.map.info.width)
            out_map.info.origin.position.x = min(self.map.info.origin.position.x,new_map.info.origin.position.x)


            meter_difference = self.map.info.origin.position.y - new_map.info.origin.position.y
            pixel_difference = int(round(meter_difference / self.map.info.resolution))

            out_map.info.height = max(new_map.info.height - pixel_difference,self.map.info.height + pixel_difference,new_map.info.height,self.map.info.height)
            out_map.info.origin.position.y = min(self.map.info.origin.position.y,new_map.info.origin.position.y)

            # if out_map.info == self.map.info:
            #     print("mapdata match!")
            #     out_map.data = self.map.data
            # else:

            # initialize a map of -1 for every square. tis results in a map with -1 in unassigned places.
            out_map.data = [-1 for i in range(0,out_map.info.width * out_map.info.height)]

            # next, copy the old map into the bigger map
            meter_difference = self.map.info.origin.position.x - out_map.info.origin.position.x
            diff_x = int(round(meter_difference / self.map.info.resolution))
            
            meter_difference = self.map.info.origin.position.y - out_map.info.origin.position.y
            diff_y = int(round(meter_difference / self.map.info.resolution))

            for m in range(0,self.map.info.width):
                for n in range(0,self.map.info.height):
                    #print("x:"+str(m)+" y:"+str(n)+" w:"+str(self.map.info.width)+" h:"+str(self.map.info.height))
                    out_map.data[m+diff_x + (n+diff_y) * out_map.info.width] = self.map.data[m + n * self.map.info.width]
            
            # now modify the data for the incoming map
            # i am only chosing to update squares that the new map contains and is at least a little bit certain about.
            meter_difference = new_map.info.origin.position.x - out_map.info.origin.position.x
            diff_x = int(round(meter_difference / self.map.info.resolution))
            
            meter_difference = new_map.info.origin.position.y - out_map.info.origin.position.y
            diff_y = int(round(meter_difference / self.map.info.resolution))
            
            gamma = 1.0/3
            for m in range(0,new_map.info.width):
                for n in range(0,new_map.info.height):
                    new_value = new_map.data[m + n * new_map.info.width]
                    if new_value != -1 and new_value != 50:
                        #print("x:"+str(m+diff_x)+" y:"+str(n+diff_y)+" w:"+str(out_map.info.width)+" h:"+str(out_map.info.height))
                        old_value = out_map.data[m+diff_x + (n+diff_y) * out_map.info.width]
                        if old_value == -1:
                            old_value = 50
                        if old_value == 0:
                            old_value = 1
                        ### MERGING FORMULA ###
                        # TODO: review asymmetry concerns, 0-handling.
                        result = int(round(old_value * math.pow(new_value*1.0/old_value,gamma)))
                        # if result == 63:
                        #     print("got a 63!")
                        #     print(old_value)
                        #     print(new_value)
                        #     print
                        out_map.data[m+diff_x + (n+diff_y) * out_map.info.width] = result
            # print(max(out_map.data))
            # print(min(out_map.data))
            return out_map
            
        def publish_map():
            self.map.header.stamp = rospy.Time.now()
            self.map_publisher.publish(self.map)
        
        # tfd_map = transform_map(new_map)
        # out_map = combine_map(tfd_map)
        out_map = combine_map(new_map)

        self.map = out_map
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
