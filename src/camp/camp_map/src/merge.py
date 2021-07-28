#!/usr/bin/env python
import math
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Pose, PoseStamped, PointStamped #Vector3, Point, PointStamped
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
            new_map = OccupancyGrid()

            # TODO: account for the possibility of different resolutions?

            # if the newq map is in the same frame as the old map, don't do anything.
            if new_map.header.frame_id == self.map.header.frame_id:
                print("same frame")
                return new_map

            # if the new map is in a different frame, try to find a transform.
            try:
                target_frame = self.map.header.frame_id
                source_frame = new_map.header.frame_id

                # Can transform?
                self.tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))

                map_transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
                # poseInNewFrame = tfgm.do_transform_pose(poseToBeTransformed, poseToPoseTransform)

                # make a new point with the header from the map
                pointin = PointStamped()
                pointin.header.frame_id = new_map.header

                # Take every corner of the map and find the maximum and minimum x and y values in the new frame
                corner_list_x = []
                corner_list_y = []
                for i in range(4):
                    # set the point to the origin of the new map
                    pointin.point = new_map.info.origin.position

                    # add a width or a height sometimes to get the other corners
                    if i == 1 or i == 3:
                        pointin.point.x = pointin.point.x + new_map.info.width*new_map.info.resolution
                    if i == 2 or i == 3:
                        pointin.point.y = pointin.point.y + new_map.info.height*new_map.info.resolution
                    
                    pointout = tfgm.do_transform_pose(pointin, map_transform)

                    corner_list_x.append(pointout.point.x)
                    corner_list_y.append(pointout.point.y)
                
                # find the largest and smallest points and use them to set the origin and h/w
                out_map = OccupancyGrid()
                out_map.info.origin.position.x = min(corner_list_x)
                out_map.info.origin.position.y = min(corner_list_y)
                out_map.info.resolution = new_map.info.resolution
                out.map.info.width = int(round((max(corner_list_x) - min(corner_list_x))/out_map.info.resolution))
                out.map.info.height = int(round((max(corner_list_y) - min(corner_list_y))/out_map.info.resolution))
                out_map.header.stamp = new_map.header.stamp
                out_map.header.frame_id = target_frame

                # now take every bit of data from the old map and add it to the new map
                out_map.data = [-1 for i in range(0,out_map.info.width * out_map.info.height)]
                for m in range(new_map.info.width):
                    for n in range(new_map.info.height):
                        # TODO: this is going to suck...
                        # transform the grid square from the original to meters
                        # then tansform it to the new frame
                        # finally switch it back to grids int he new frame
                        # then write the data in the new map.
                        # maybe make a square2meter([x,y]map.info) function?

            # If the transform cannot occur (an exception has been raised), catch it, and sleep.
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Transform Problem!!")
                rospy.sleep(1)



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
