import math
import roslib
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid

import tf2_ros as tfr
import tf2_geometry_msgs as tfgm

def transform_map(in_map,out_frame,tf_buffer):
    # Declaration to allow tab-completion 
    # in_map = OccupancyGrid()

    # TODO: account for the possibility of different resolutions?

    # if the new map is in the same frame as the old map, don't do anything.
    if in_map.header.frame_id == out_frame:
        print("same frame")
        return in_map
        
    print("different frame")

    # if the new map is in a different frame, try to find a transform.
    try:
        target_frame = out_frame
        source_frame = in_map.header.frame_id
        print("target:")
        print(target_frame)
        print("source:")
        print(source_frame)

        # Can transform?
        tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))

        map_transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
        # poseInNewFrame = tfgm.do_transform_pose(poseToBeTransformed, poseToPoseTransform)

        # make a new point with the header from the map
        pointin = PointStamped()
        pointin.header.frame_id = in_map.header

        # Take every corner of the map and find the maximum and minimum x and y values in the new frame
        corner_list_x = []
        corner_list_y = []
        for i in range(4):
            # set the point to the origin of the new map
            pointin.point = in_map.info.origin.position

            # add a width or a height sometimes to get the other corners
            if i == 1 or i == 3:
                pointin.point.x = pointin.point.x + in_map.info.width*in_map.info.resolution
            if i == 2 or i == 3:
                pointin.point.y = pointin.point.y + in_map.info.height*in_map.info.resolution
            
            pointout = tfgm.do_transform_point(pointin, map_transform)

            corner_list_x.append(pointout.point.x)
            corner_list_y.append(pointout.point.y)
        
        # find the largest and smallest points and use them to set the origin and h/w
        out_map = OccupancyGrid()
        out_map.info.origin.position.x = min(corner_list_x)
        out_map.info.origin.position.y = min(corner_list_y)
        out_map.info.resolution = in_map.info.resolution
        out_map.info.width = int(round((max(corner_list_x) - min(corner_list_x))/out_map.info.resolution))
        out_map.info.height = int(round((max(corner_list_y) - min(corner_list_y))/out_map.info.resolution))
        out_map.header.stamp = in_map.header.stamp
        out_map.header.frame_id = target_frame

        # now take every bit of data from the old map and add it to the new map
        # first, make a PointStamped so we don't make one every loop.
        new_point = PointStamped()
        new_point.header.frame_id = in_map.header.frame_id
        new_point.header.stamp = in_map.header.stamp
        # initialize the output to all -1
        out_map.data = [-1 for i in range(0,out_map.info.width * out_map.info.height)]
        # for each input data point
        for i in range(0,in_map.info.width * in_map.info.height):
            # if the index has useful information
            if in_map.data[i] != -1:
                # find the index's location in meters (point form)
                new_point.point = index2point(i,in_map.info)
                # transform the point to the new frame
                out_point = tfgm.do_transform_point(new_point, map_transform)
                # find the index for the point on the output map
                out_index = point2index(out_point.point,out_map.info)
                # save the data in the output map
                if out_map.data[out_index] == -1:
                    # Normal case where this is the first data written to a square
                    out_map.data[out_index] = in_map.data[i]
                else:
                    # possible case where two data points are assigned to the same square
                    print("double writing is a case that needs consideration")
                    out_map.data[out_index] = (in_map.data[i] + out_map.data[out_index])/2
        
        return out_map


    # If the transform cannot occur (an exception has been raised), catch it, and sleep.
    except (tfr.LookupException, tfr.ConnectivityException, tfr.ExtrapolationException) as e:
        print("Transform Problem!!")
        print(e)
        rospy.sleep(1)
        return -1

def index2point(index,info):
    #info = OccupancyGrid().info
    x = index % info.width
    y = index // info.width
    point = Point()
    point.x = x*info.resolution + info.origin.position.x
    point.y = y*info.resolution + info.origin.position.y
    point.z = 0
    return point

def point2index(point,info):
    x = point.x
    y = point.y
    x = int(round((x - info.origin.position.x)/info.resolution))
    y = int(round((y - info.origin.position.y)/info.resolution))
    index = x + y * info.width
    return index

# This function combines the new map's data with the existing map.
def combine_map(in_map,old_map):
    # Declaration to allow tab-completion 
    #in_map = OccupancyGrid()
    #! Assuming rotation is taken care of in 
    # first, make sure we have an output map big enough to hold all the data based off of old_map
    # make sure to set the map.info to match

    # copy the data from the existing saved map into a new map
    out_map = OccupancyGrid()
    out_map.header = in_map.header
    out_map.info.resolution = old_map.info.resolution

    # check to see if the new map has an origin further to the bottom corner than the saved one.
    # assign a new origin and width/height
    meter_difference = old_map.info.origin.position.x - in_map.info.origin.position.x
    pixel_difference = int(round(meter_difference / old_map.info.resolution))

    out_map.info.width = max(in_map.info.width - pixel_difference,old_map.info.width + pixel_difference,in_map.info.width,old_map.info.width)
    out_map.info.origin.position.x = min(old_map.info.origin.position.x,in_map.info.origin.position.x)


    meter_difference = old_map.info.origin.position.y - in_map.info.origin.position.y
    pixel_difference = int(round(meter_difference / old_map.info.resolution))

    out_map.info.height = max(in_map.info.height - pixel_difference,old_map.info.height + pixel_difference,in_map.info.height,old_map.info.height)
    out_map.info.origin.position.y = min(old_map.info.origin.position.y,in_map.info.origin.position.y)

    # if out_map.info == old_map.info:
    #     print("mapdata match!")
    #     out_map.data = old_map.data
    # else:

    # initialize a map of -1 for every square. tis results in a map with -1 in unassigned places.
    out_map.data = [-1 for i in range(0,out_map.info.width * out_map.info.height)]

    # next, copy the old map into the bigger map
    meter_difference = old_map.info.origin.position.x - out_map.info.origin.position.x
    diff_x = int(round(meter_difference / old_map.info.resolution))
    
    meter_difference = old_map.info.origin.position.y - out_map.info.origin.position.y
    diff_y = int(round(meter_difference / old_map.info.resolution))

    for m in range(0,old_map.info.width):
        for n in range(0,old_map.info.height):
            #print("x:"+str(m)+" y:"+str(n)+" w:"+str(old_map.info.width)+" h:"+str(old_map.info.height))
            out_map.data[m+diff_x + (n+diff_y) * out_map.info.width] = old_map.data[m + n * old_map.info.width]
    
    # now modify the data for the incoming map
    # i am only chosing to update squares that the new map contains and is at least a little bit certain about.
    meter_difference = in_map.info.origin.position.x - out_map.info.origin.position.x
    diff_x = int(round(meter_difference / old_map.info.resolution))
    
    meter_difference = in_map.info.origin.position.y - out_map.info.origin.position.y
    diff_y = int(round(meter_difference / old_map.info.resolution))
    
    gamma = 1.0/3
    for m in range(0,in_map.info.width):
        for n in range(0,in_map.info.height):
            new_value = in_map.data[m + n * in_map.info.width]
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