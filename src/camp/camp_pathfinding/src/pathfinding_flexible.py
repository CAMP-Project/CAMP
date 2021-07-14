#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 4/15/2021
# Description     : This code is adapted from an algorithm developed by Dr. Michael 
#                   McCourt. This code uses LiDAR and probabilistic map data to 
#                   generate waypoints for robotic pathfinding. 
#
#
#
# Change Log:
#
# 4/15/2021 - Sean Carda: Created script and initial methods.
#
# 5/5/2021  - Sean Carda: Finalized the code. It is entirely possible that more
#                         edits will be necessary.
#
# 6/25/2021 - Tyler Pigott: Tried to fix the map extention error, did too much
#
# 7/13/2021 - Tyler Pigott: TODO:Time to change everything to the decawave system
#
#----------------------------------------------------------------------------------------


# FOR DEVELOPMENT
# ---------------
# This code will subscribe to multiple Turtlebot topics such as its odometry and map.
# This code will use odometry to localize itself within the map. It will use map
# data to determine where to scan and where to generate waypoints.
# The current functionality outline of this code is as follows (this is subject to change):
# 1. Initialize a set of waypoints in one direction.
# 2. Check lidar distances.
# 3a. If no objects are detected to impede movement, travel to waypoint, shift waypoints, and create new waypoint.
# 3b. If an object is detected, reset the waypoint list. 

# Package imports.
import math
import roslib
import rospy
import time

import numpy as np
import cv2 as cv

from rospy.core import rospyinfo
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Vector3, Point, PointStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from camp_goto.msg import Cmd

class Waypoint:
    def __init__(self,pub='none',x=0,y=0,theta=0):
        self.point = Point(x,y,0)
        self.heading = theta
        self.viz_publisher = pub
        # self.viz_publisher = rospy.Publisher('waypoint_'+str(num), PointStamped, queue_size = 10)

    # def __del__(self):
    #     self.viz_publisher.unregister()

    
    def __str__(self):
        return "Point "+str(self.num)+":("+str(self.point.x)+","+str(self.point.y)+") @ "+str(self.heading)
        
    def publish(self):
        if isinstance(self.viz_publisher, rospy.Publisher):
            # Result as a PointStamp.
            result_viz = PointStamped()
            result_viz.point = self.point
            result_viz.header.stamp = rospy.Time()
            result_viz.header.frame_id = 'deca'
            self.viz_publisher.publish(result_viz)

    def auto_adjust(self,map):
        # convert point from meters to grid squares (i don't int(round()) this because i do that in the l/r check)
        x = (self.point.x - map.info.origin.position.x)/map.info.resolution
        y = (self.point.y - map.info.origin.position.y)/map.info.resolution
        # set a unit vector normal to the heading
        unit_x = math.cos(self.heading + math.pi/2)
        unit_y = math.sin(self.heading + math.pi/2)
        # initialize left and right at 1m (20 squares)
        left = 20
        right = 20
        # if there is a wall closer than 1m, write the number of grid squares to left or right.
        # TODO: handle edge of map cases?
        for i in range(0,20):
            if left == 20 and map.data[int(round(x + i * unit_x)) + (map.info.width * int(round(y + i * unit_y)))] > 90:
                left = i
            if right == 20 and map.data[int(round(x - i * unit_x)) + (map.info.width * int(round(y - i * unit_y)))] > 90:
                right = i
        # find out how many meters we need to move the point to get it 0.5m from the wall
        adjust = (left - right)/2 * map.info.resolution
        # create adjustment vectors in meters for the x and y directions
        adjust_x = (unit_x * adjust)
        adjust_y = (unit_y * adjust)
        # change the point to the adjusted point
        self.point.x = self.point.x + adjust_x
        self.point.y = self.point.y + adjust_y
        #print("points are " + str(left) + " and " + str(right) + " squares away on the left and right respectively")
        print("adjusted the point by "+str(adjust_x)+" x and "+str(adjust_y)+" y.")
        return [adjust_x,adjust_y]
    
    def quick_adjust(self,adjust):
        self.point.x = self.point.x + adjust[0]
        self.point.y = self.point.y + adjust[1]
        #print("executed quick adjust")



class Pathfinding_Node:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # number of directions to look when deciding on a direction
        self.direction_count = 12
        #number of waypoints to generate
        self.waypoint_count = 4

        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('pathfinding', anonymous = False)

        # Subscribe to map.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)      
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to odometry.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # Subscribe to backup state.
        rospy.Subscriber('/backup_state', Bool, self.updateBackup)

        # This will publish the computed waypoint information.
        self.point_publisher = rospy.Publisher('go_cmd', Cmd, queue_size = 10)
        
        # Instantiate publlishers for displaying the waypoints in rviz. This will be invaluable for debugging.
        # these moved to the new waypoint class
#         self.viz_publisher_1 = rospy.Publisher('point_viz_1', PointStamped, queue_size = 10)
#         self.viz_publisher_2 = rospy.Publisher('point_viz_2', PointStamped, queue_size = 10)
#         self.viz_publisher_3 = rospy.Publisher('point_viz_3', PointStamped, queue_size = 10)
#         self.viz_publisher_4 = rospy.Publisher('point_viz_4', PointStamped, queue_size = 10)
        # self.pubs = [rospy.Publisher('waypoint_0', PointStamped, queue_size = 10),rospy.Publisher('waypoint_1', PointStamped, queue_size = 10),rospy.Publisher('waypoint_2', PointStamped, queue_size = 10),rospy.Publisher('waypoint_3', PointStamped, queue_size = 10)]
        self.pubs = [rospy.Publisher('waypoint_'+str(i), PointStamped, queue_size = 10) for i in range(0,self.waypoint_count)]
        self.region_publisher = rospy.Publisher('region', PointStamped, queue_size = 10)
        self.robot_publisher = rospy.Publisher('robot_publisher', PointStamped, queue_size = 10)
        

        # make waypoint_count number of waypoints
        self.waypoints = [Waypoint('none')]

        # Initialize important data types. For this script, we need access to the OccupancyGrid produced
        # by SLAM. We need the Lidar for obstacle detection. We need the odometry for positional data. 
        self.mapActual = OccupancyGrid() 
        self.lidar = LaserScan()                         
        self.odom = Odometry()

        # This is a check to prevent the robot from conducting too many reset calculations at once.
        self.reset = False
        
        # Track number of fails during a new waypoint calculation.
        self.fails = 0

        # Property which holds on to the satisfactory distance for when a new point should be generated.
        self.satisDist = 0.15 #meters

        self.entropyVector = [0, 0, 0, 0, 0, 0, 0, 0]

        self.last_backup = False

        self.backup = False


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.mapActual = data

    # This method will update lidar data when new data will be available. This method grabs every parameter 
    # from the lidar node.
    def updateLidarScan(self, data):
        self.lidar = data

    # This method will grab information from the robot's odometry.
    def updateOdom(self, data):
        self.odom = data

    # Callback to update the backup state.
    def updateBackup(self, data):
        self.backup = data.data

    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the Pathfinding algorithm
    #--------------------------------------------------------------------------------------------------------------
    def main(self):        
    
            # Method for obtaining the robot's position as a distance, in meters, relative to the SLAM-generated map.
        def getRoboMapPosition():
            # Use Odometry to get the robot's position.
            result = Vector3()
            result.x = self.odom.pose.pose.position.x
            result.y = self.odom.pose.pose.position.y

            # Return.
            return result


        # Method for obtaining the robot's position as a matrix position in the SLAM-generated map.
        def getMatrixPosition():
            x_map_offset = -1 * self.mapActual.info.origin.position.x # X origin of map object is -10. So, add 10 to robot X position.
            y_map_offset = -1 * self.mapActual.info.origin.position.y # Y origin of map object is -10. So, add 10 to robot Y position.
            
            # Get the robot position relative to the map
            robo_position = getRoboMapPosition()

            # Calculate the robot's position in meters from map object origin.
            position_in_meters = [x_map_offset + robo_position.x, 
                                  y_map_offset + robo_position.y]

            # Convert to matrix position. Used for map data traversing.
            position_in_units = [0, 0]
            while self.mapActual.info.resolution == 0:
                print('wait a sec...')
                rospy.sleep(0.2)
            for i in range(2):
                position_in_units[i] = round(position_in_meters[i] / self.mapActual.info.resolution, 0)
                position_in_units[i] = int(position_in_units[i])

            # Return.
            return position_in_units


        # Method for publishing waypoints to RQT. 
        def publishWaypoints():

            command = Cmd()
            try:
                command.destination = self.waypoints[0].point
            except:
                print("destination publish failed")
                printWaypoints()
                exit()
            command.stop = False
            command.is_relative = False
            command.is_deca = True

            command.speed = 0.43

            command.destination_stop_distance = 0
            command.emergency_stop_distance = 0.15
            command.emergency_stop_angle = 30

            self.point_publisher.publish(command)

            # Publish the points in rviz.
            #print("publishing...")
            for w in self.waypoints:
                w.publish()


            roboPosX = getRoboMapPosition().x
            roboPosY = getRoboMapPosition().y
            roboPos = PointStamped()
            roboPos.header.stamp = rospy.Time()
            roboPos.header.frame_id = 'deca'
            roboPos.point = Point(roboPosX, roboPosY, 1)

            self.robot_publisher.publish(roboPos)

        # This method replaces the resetWaypoints() function. It searches for a point of interest to travel to or uses the traditional reset function if a point of interest is already close by.
        def reevaluate():
            print("reevaluating...")
            # prep the data
            while self.mapActual.info.resolution == 0:
                print('wait a sec...')
                rospy.sleep(0.2)

            # get the map of unknown by free areas
            map = np.array(self.mapActual.data).reshape(self.mapActual.info.height, self.mapActual.info.width, order='C')
            free = cv.inRange(map,0,20)
            free = free//255
            wall = cv.inRange(map,80,100)
            wall = wall//255
            print(free)
            print(wall)

            # perform opening before dialating the map
            smallfree = cv.erode(free,np.ones((3,3), np.uint8))
            openfree = cv.dilate(smallfree,np.ones((3,3), np.uint8))

            bigfree = cv.dilate(openfree,np.ones((3,3), np.uint8))
            bigwall = cv.dilate(wall,np.ones((7,7), np.uint8))
            print(bigfree)
            print(bigwall)
            
            wallorfree = cv.bitwise_or(bigwall,free)
            notwallorfree = cv.bitwise_not(wallorfree)
            notwallorfree = notwallorfree//255
            poimap = cv.bitwise_and(bigfree, bigfree, mask = notwallorfree)
            print(wallorfree)
            print(notwallorfree)
            print(poimap)
            


            #poilist = np.where(poimap == 1)

            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y

            # look for areas with many 1s on the poimap. we will start in a small radius and branch further
            nearmap = poimap[meter2grid(y-0.5,'y'):meter2grid(y+0.5,'y'),meter2grid(x-0.5,'x'):meter2grid(x+0.5,'x')]
            nearsum = np.sum(nearmap)
            print("how cool is the local square meter?")
            print(x)
            print(y)
            print(nearmap)
            print(nearsum)
            if nearsum >= 5:
                print("keep looking")
                resetWaypoints()
            else:
                print("time for a change")
                lastrad = 0
                rad = 5
                max_poi = 0
                width = self.mapActual.info.width
                height = self.mapActual.info.height

                while max_poi < 1 and rad < max(width,height)*1.8:
                    for m in range(-rad,rad+1):
                        for n in range(-rad,rad+1):
                            dist = m*m+n*n
                            if dist <= rad and dist > lastrad:
                                print("@: ("+str(m+x)+","+str(n+y)+")")
                                poi_submap = poimap[meter2grid(y-0.5+n,'y'):meter2grid(y+0.5+n,'y'),meter2grid(x-0.5+m,'x'):meter2grid(x+0.5+m,'x')]
                                print(poi_submap)
                                poi_count = np.sum(poi_submap)
                                poi_score = min(poi_count,10) * (0.5 + 0.5 * math.exp(-dist*4/25))
                                if poi_score > max_poi:
                                    # take the highest score and set it as the target
                                    max_poi = poi_score
                                    print("found max_poi")
                                    print(max_poi)
                                    max_x = x+m
                                    max_y = y+n
                                    print("("+str(max_x)+","+str(max_y)+")")
                                #time.sleep(1)
                    #exit()
                    lastrad = rad
                    rad = rad + 5

                # now that a point of interest has been identified, we need to make waypoints leading to that POI.
                # for now, lets assume the point is on navigable terrain.
                # we will now pass the start and end info to our recursive pathfinding algorithm

                # make the map a 2d array
                map = np.array(self.mapActual.data, np.uint8).reshape(self.mapActual.info.height, self.mapActual.info.width, order='C')
                freemap = cv.inRange(map,0,20)
                kernel = np.ones((7,7), np.uint8)
                freemap = cv.erode(freemap,kernel)
                
                path = makePath([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,max_x,max_y],freemap)

                self.waypoints = [Waypoint('none') for i in range(1,len(path)/2)]
                print("all waypoints deleted")
                lx = path[0]
                ly = path[1]
                for i in range(1,len(path)/2):
                    x = path[2*i]
                    y = path[2*i+1]
                    theta = math.atan2(y-ly,x-lx)
                    print("new path Waypoint")
                    print(i)
                    if i-1 < self.waypoint_count:
                        self.waypoints[i-1] = Waypoint(self.pubs[i-1],x,y,theta)
                    else:
                        self.waypoints[i-1] = Waypoint('none',x,y,theta)
                    print("waypoint length")
                    print(len(self.waypoints))
                    lx = x
                    ly = y

                    #TODO: make a bunch of waypoints, requires a review of waypoint infrastructure to allow for waypoint surges.
                # TODO:                
                #  if the target is unreachable, mark it as "explored"
                # when you get to the target, set it's square as explored.
                
        def printWaypoints():
            for w in self.waypoints:
                print(w)

        def makePath(path,map):
            print("/!\ making a path /!\\")
            print(path)
            size = int(1/self.mapActual.info.resolution)
            width = self.mapActual.info.width
            height = self.mapActual.info.height
            # get the data and convert to grid squares
            sx = path[0]
            sy = path[1]
            gx = path[2]
            gy = path[3]
            distance = math.sqrt((sx-gx)*(sx-gx)+(sy-gy)*(sy-gy))
            if distance < 0.2:
                # if endponts are already super close together, stop.
                return path

            # save a unit vector for ease of use later
            dx = (gx-sx)/distance
            dy = (gy-sy)/distance
            # search for non-free space in the path. 
            # Don't look too close to the POI because it could be in a wall or something
            wall = 0
            for i in range(0,int(round((distance-1)*size))):
                if map[(meter2grid(sy,'y')+int(round(dy*i)))][meter2grid(sx,'x')+int(round(dx*i))] == 0:
                    wall = 1
                    break
            if wall == 0:
                return path

            # expand each map until they overlap, 
            # then set the overlap as an intermediary waypoint and repeat on both sides
            # startmap = []
            # goalmap = []
            # startmap = [0 for i in range(0,width*height)]
            # goalmap = [0 for i in range(0,width*height)]
            startmap = np.zeros((height, width), np.uint8)
            goalmap = np.zeros((height, width), np.uint8)
            # startmap[meter2grid(sx,'x') + meter2grid(sy,'y')*width] = 1
            # goalmap[meter2grid(gx,'x') + meter2grid(gy,'y')*width] = 1
            startmap[meter2grid(sy,'y')][meter2grid(sx,'x')] = 1
            goalmap[meter2grid(gy,'y')][meter2grid(gx,'x')] = 1

            # make goalmap poi bigger
            goalmap = cv.dilate(goalmap,cv.getStructuringElement(cv.MORPH_ELLIPSE,(25,25)))
            goalsum = np.sum(goalmap)
            print("goalsum: "+str(goalsum))

            val = 0
            overlap = 0
            # build a circular kernel real quick
            # kernel_list = []
            # kernel_list = [0 for i in range(7*7)]
            # for i in range(7*7):
            #     if (i%7-3)*(i%7-3) + (int(i/7)-3)*(int(i/7)-3) < 12:
            #         kernel_list[i] = 1
            # kernel = np.array(kernel_list).reshape(7,7,order='C')
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(11,11))

            # keep looping until an overlap is found or the maps no longer change
            gen = 0
            while overlap == 0:
                gen = gen + 1
                print("gen:"+str(gen))
                startmap = cv.dilate(startmap,kernel)
                goalmap = cv.dilate(goalmap,kernel)
                goalsum = np.sum(goalmap)
                print("goalsum: "+str(goalsum))

                # print(startmap)
                # print(goalmap)
                # print(map)

                # TODO: something here is broken
                startmap = cv.bitwise_and(startmap,startmap,mask=map)
                goalmap = cv.bitwise_and(goalmap,goalmap,mask=map)
                # print(np.where(startmap == 1))

                overlapmap = cv.bitwise_and(goalmap,goalmap,mask=startmap)
                print("overlap: "+str(np.sum(overlapmap)))
                if np.sum(overlapmap) >= 1:
                    overlap = 1
                    newpoint = np.where(overlapmap == 1)
                    mx = grid2meter(newpoint[1][0],'x')
                    my = grid2meter(newpoint[0][0],'y')
                    
                startsum = np.sum(startmap)
                print("startsum: "+str(startsum))
                goalsum = np.sum(goalmap)
                print("goalsum: "+str(goalsum))
                if goalsum == 0:
                    print(map[meter2grid(gy-0.5,'y'):meter2grid(gy+0.5,'y'),meter2grid(gx-0.5,'x'):meter2grid(gx+0.5,'x')])
                    print(str(gx)+","+str(gy))
                    print("goalsum fail")
                    exit()
                newval = startsum + goalsum
                if newval == val:
                    # something needs to happen here so the POI gets shrown out, but i don't know how yet. maybe errors or exceptions?
                    print("no path available")
                    exit()
                    return
                val = newval
            
            # handle recursion
            firsthalf = makePath([sx,sy,mx,my],map)
            secondhalf = makePath([mx,my,gx,gy],map)
            path = firsthalf + [secondhalf[i] for i in range(2,len(secondhalf))]
            print(path)
            return path

        
        # this function converts meters to grid squares
        def meter2grid(n,param='none'):
            if param == 'x':
                return int(round((n - self.mapActual.info.origin.position.x) / self.mapActual.info.resolution))
            if param == 'y':
                return int(round((n - self.mapActual.info.origin.position.y) / self.mapActual.info.resolution))
            if param == 'none':
                return int(round(n / self.mapActual.info.resolution))

        #this function does the opposite
        def grid2meter(n,param='none'):
            if param == 'x':
                return self.mapActual.info.origin.position.x + n * self.mapActual.info.resolution
            if param == 'y':
                return self.mapActual.info.origin.position.y + n * self.mapActual.info.resolution
            if param == 'none':
                return int(round(n * self.mapActual.info.resolution))

        # This method resets the waypoints in the event of an obstacle preventing the traversal
        # to waypoint 1 or if all paths fail around waypoint 3.
        def resetWaypoints():
            print("reset")
            # TODO: modify to match waypoint generation (i didn't plan this one out yet)
            # Get position of robot as a matrix value in the map.
            #x_pos = int(getMatrixPosition()[0])
            #y_pos = int(getMatrixPosition()[1])

            x_pos = getMatrixPosition()[0]
            y_pos = getMatrixPosition()[1]

            dist = 0.3

            entropyDirections = []
            entropyDirections = [0 for i in range(0,self.direction_count)]
            foundWalls = []
            foundWalls = [False for i in range(0,self.direction_count)]

            rayLimit = 40
            offset = 30
            power = 2

            if x_pos > self.mapActual.info.origin.position.x and y_pos > self.mapActual.info.origin.position.y:
                print("xpos>xmin,ypos>ymin")
                # Calculate entropy sums.
                for k in range(1, rayLimit):
                    #print("k = " + str(k))
                    # Check each direction
                    for d in range(0,self.direction_count):
                        x = (x_pos + k*math.cos(2*math.pi/self.direction_count * d))
                        y = (y_pos + k*math.sin(2*math.pi/self.direction_count * d))
                        if x > 0 and x < self.mapActual.info.width and y > 0 and y < self.mapActual.info.height and foundWalls[d] is False:
                            #print(str(x)+" , "+str(y))
                            mapdata = map(int(round(x)),int(round(y)))
                            if mapdata > 90:
                                foundWalls[d] = True
                            entropyDirections[d] = entropyDirections[d] + pow(mapdata - offset, power)/k
                        else:
                            entropyDirections[d] = entropyDirections[d] + 9999
                
            # Find the direction of minimum entropy.
            direction = None
            maxVal = 1000
            minVal = 500
            self.entropyVector = entropyDirections
            #for value in entropyDirections:
            #    if value < maxVal and value > minVal:
            #        direction = entropyDirections.index(value)
            #        break
            
            #if direction is None:
            direction = entropyDirections.index(min(entropyDirections))

            # Establish path as a series of new waypoints.
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            self.waypoints = [Waypoint('none') for i in range(0,self.waypoint_count)]
            for i in range(0,self.waypoint_count):
                x = x + dist*math.cos(2*math.pi/self.direction_count * direction)
                y = y + dist*math.sin(2*math.pi/self.direction_count * direction)
                self.waypoints[i] = Waypoint(self.pubs[i],x,y,2*math.pi/self.direction_count * direction)


        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():
            print("new waypoint!")
            # Get the position of the last waypoint.
            if len(self.waypoints)>0:
                end = self.waypoints[len(self.waypoints)-1].point
            else:
                end = Point()
                end.x = self.odom.pose.pose.position.x
                end.y = self.odom.pose.pose.position.y

            # number of directions
            #self.direction_count = 6
            # distance from last point in meters
            dist = 0.4
            # height and width of square to test in meters
            size = 0.5
            biggerboxmultiplier = 3

            entropyDirections = []
            entropyDirections = [0 for i in range(0,self.direction_count)]
            for i in range(0,self.direction_count):
                #print(i)
                #entropyDirections[i] = grabEntropySquare(end.x + math.cos(2*math.pi/self.direction_count * i)*dist - size/2, end.x + math.cos(2*math.pi/self.direction_count * i)*dist + size/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist - size/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist + size/2)
                #+ grabEntropySquare(end.x + math.cos(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier - size*biggerboxmultiplier/2, end.x + math.cos(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier + size*biggerboxmultiplier/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier - size*biggerboxmultiplier/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier + size*biggerboxmultiplier/2)/9*0.1
                entropyDirections[i] = grabEntropyCircle(size/2, end.x + math.cos(2*math.pi/self.direction_count * i)*dist, end.y + math.sin(2*math.pi/self.direction_count * i)*dist)
                #TODO: add more boxes
            # Initialize a parameter to check if there is an obstacle between the 3rd waypoint and the generated waypoint.
            # It is assumed to be true that there is an obstacle between the points.
            isObstacle = 1

            while isObstacle == 1:
                # inc = 0
                # #test = 0
                print("entropy values:")
                for x in entropyDirections:
                    print(x)
                #     inc = inc + 1

                # Find the direction to place a new waypoint. The square with the highest entropy is chosen.
                # The entropy tells the robot where the "highest reward" is.
                # TODO: implement tiebreaker?
                direction = entropyDirections.index(max(entropyDirections))   

                #rospy.loginfo("try:"+str(direction))           

                # Get the number of squares to increase in either direction depending on the calculated direction.
                dx = dist*math.cos(2*math.pi/self.direction_count * direction)
                dy = dist*math.sin(2*math.pi/self.direction_count * direction)

                # Check for points within one dist
                duplicates = 0
                for i in range(0, len(self.waypoints)):
                    #rospy.loginfo("Point " + str(i) +"\nNew point: ("+str(end.x + dx)+","+str(end.y + dy)+")\nOld point: ("+str(self.waypoints.get(i).x)+","+str(self.waypoints.get(i).y)+")") 
                    if math.sqrt(math.pow(end.x + dx - self.waypoints[i].point.x,2) + math.pow(end.y + dy - self.waypoints[i].point.y,2)) < dist:
                        #rospy.loginfo("Match")     
                        duplicates = duplicates + 1
                    #else:
                        #rospy.loginfo("No Match")

                # Connect the 3rd point and the theoretical last point.
                yMin = min([end.y, end.y + dy])
                yMax = max([end.y, end.y + dy])
                xMin = min([end.x, end.x + dx])
                xMax = max([end.x, end.x + dx])

                # Check for bounds errors or if any duplicates exist. If so, restart the sweek by setting the entropy
                # sum in that direction to 0. This will prevent that direction from being searched again since the 
                # algorithm checks for the maximum entropy value.
                #print("("+str(direction)+")")
                if yMin < self.mapActual.info.origin.position.y + 1 or yMax >= self.mapActual.info.origin.position.y + self.mapActual.info.height*self.mapActual.info.resolution -1 or xMin < self.mapActual.info.origin.position.x + 1 or xMax >= self.mapActual.info.origin.position.x + self.mapActual.info.width*self.mapActual.info.resolution -1 or duplicates > 0:
                    # if duplicates > 0:
                    #     rospy.loginfo("duplicates > 0 (" + str(duplicates) + ")")
                    entropyDirections[direction] = 0
                else:
                    maximum = 0
                    for i in range(int(round((xMin - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution) - 1), int(round((xMax - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution) + 2)):
                        for j in range(int(round((yMin - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution) - 1), int(round((yMax - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution) + 2)):
                            regionX = (xMax + xMin)/2
                            regionY = (yMax + yMin)/2
                            regionPos = PointStamped()
                            regionPos.header.stamp = rospy.Time()
                            regionPos.header.frame_id = 'deca'
                            regionPos.point = Point(regionX, regionY, 1)
                            self.region_publisher.publish(regionPos)
                            if map(i, j) > maximum:
                                maximum = map(i, j)
                    if maximum > 70:
                        entropyDirections[direction] = 0
                        #rospy.loginfo("maximum > 70 (" + str(maximum) + ")")
                    else:
                        self.waypoints = self.waypoints + [Waypoint(len(self.waypoints),end.x + dx, end.y + dy,2*math.pi/self.direction_count * direction)]
                        if len(self.waypoints) < self.waypoint_count:
                            self.waypoints = self.waypoints + [Waypoint(self.pubs[len(self.waypoints)],end.x + dx, end.y + dy,2*math.pi/self.direction_count * direction)]
                        else:
                            self.waypoints = self.waypoints + [Waypoint('none',end.x + dx, end.y + dy,2*math.pi/self.direction_count * direction)]
                        #printWaypoints()
                        isObstacle = 0
                        self.fails = 0

                # Track number of fails.
                self.fails = self.fails + 1
                #rospy.loginfo("Fails: " + str(self.fails))

                if self.fails > self.direction_count:
                    print("fail reset")
                    reevaluate()
                    self.fails = 0
                    isObstacle = 0

        def shiftWaypoints():
            # print("before the shift")
            # printWaypoints()
            for i in range(0,len(self.waypoints)-1):
                if i < self.waypoint_count:
                    self.waypoints[i] = Waypoint(self.pubs[i],self.waypoints[i+1].point.x,self.waypoints[i+1].point.y,self.waypoints[i+1].heading)
                else:
                    self.waypoints[i] = Waypoint('none',self.waypoints[i+1].point.x,self.waypoints[i+1].point.y,self.waypoints[i+1].heading)
                
            self.waypoints = [self.waypoints[i] for i in range(0,len(self.waypoints)-1)]
            # print("after the shift")
            # printWaypoints()
            #time.sleep(0.5)


        # Method to calculate an entire region of entropy. Reduces the necessary lines of code to write.
        def grabEntropySquare(range_x_1, range_x_2, range_y_1, range_y_2):
            # Initialize the result of the scan.
            result = 0
            # convert all inputs to grid domain
            while self.mapActual.info.resolution == 0:
                print('wait a sec...')
                rospy.sleep(0.2)
            range_x_1 = int(round((range_x_1 - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution))
            range_x_2 = int(round((range_x_2 - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution))
            range_y_1 = int(round((range_y_1 - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution))
            range_y_2 = int(round((range_y_2 - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution))
            area = (-min(range_x_1, range_x_2)+max(range_x_1, range_x_2) + 1) * (-min(range_y_1, range_y_2)+max(range_y_1, range_y_2) + 1)

            # On the bounds of the given entropy region, calculate the total entropy.
            # added some extra brain to make sure we go biggest to smallest and to make the last value inclusive.
            for i in range(min(range_x_1, range_x_2),max(range_x_1, range_x_2) + 1):
                for j in range(min(range_y_1, range_y_2),max(range_y_1, range_y_2) + 1):
                    result = result + entropy(i, j)

            # Return.
            return result/area


        # Method to calculate a circular region of entropy. 
        def grabEntropyCircle(radius, center_x, center_y):
            radius = meter2grid(radius)
            center_x = meter2grid(center_x,'x')
            center_y = meter2grid(center_y,'y')
            
            # Establish a reasonable set of bounds in which to examine data points. We do not
            # want to have to search through the entire occupancy grid for values. Expecially
            # if the occupancy grid becomes very large.
            bound_x_left = center_x - radius
            bound_x_right = center_x + radius
            bound_y_down = center_y - radius
            bound_y_up = center_y + radius
            result = 0
            area = 0

            # In the bounds we created, determine if a point lies within the inscribed circle defined by
            # the given radius and center.
            for i in range(bound_x_left, bound_x_right + 1):
                for j in range(bound_y_down, bound_y_up + 1):
                    test = math.sqrt(pow(i - center_x, 2) + pow(j - center_y, 2))
                    #print(test)
                    if test < radius:
                        area = area + 1
                        result = result + entropy(i, j)

            # Return result.
            return result/area
                

        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            # If the value at a given index is -1, return 100. This is to keep the robot from travrsing
            # to regions that have not been explored.
            # rospy.loginfo(len(self.mapActual.data))
            #print("checking ("+str(x)+","+str(y)+")")
            if self.mapActual.data[x + (self.mapActual.info.width * (min(y,self.mapActual.info.height-1)))] < 0:
                return 50
            # Return.
            else:
                return self.mapActual.data[x + (self.mapActual.info.width * y)]

        # Method to calculate the entropy at a given map index.
        def entropy(x, y):
            #print("entropy("+str(x)+","+str(y)+")")
            # First grab probability. Divide by 100.
            try:
                p = self.mapActual.data[x + (self.mapActual.info.width * (min(y,self.mapActual.info.height-1)))]/100.0
            except:
                print("error: tried to access ("+str(x)+","+str(y)+") and failed")
                p = 0.5
            
            # If the value of the probability at the given index is negative, replace it with 0.5.
            # Note: this does not replace the value of the probability value in the OccupancyMap.
            if p < 0:
                p = 0.5

            # Quick calculation to ensure that the probability is between 0.01 and 0.99.
            p = (p * 0.98) + 0.01

            # Return.
            return ((-p * math.log(p, 2)) - ((1 - p) * math.log(1 - p, 2)))


        # This method checks for obstacles between the robot and waypoint 1. Taken from camp_goto_node.
        def obstacleCheck():
            # Instantiate a closest object integer.
            closestFrontObject = 0

            # Obtain lidar distance data.
            ranges = self.lidar.ranges
            if len(ranges) == 360:
                if ranges[0] != 0:
                    closestFrontObject = ranges[0]
                else:
                    closestFrontObject = 500

                # Increment the second value in this loop to sweep over a larger angle.
                for i in range(1, 17):
                    if ranges[i] < closestFrontObject and ranges[i] != 0:
                        closestFrontObject = self.lidar.ranges[i]
                    if ranges[360 - i] < closestFrontObject and ranges[360 - i] != 0:
                        closestFrontObject = ranges[360 - i]
                
                # Threshold distance. *This double is in meters.
                if closestFrontObject < 0.3:
                    print("object "+str(closestFrontObject)+" meters away!")
                    return True
                else:
                    return False
            else:
                print("no lidar data!")
                return True
        
        # Main functionality of the pathfinding code. 
        done = 0
        while self.mapActual.info.resolution == 0 :
            print('wait a sec...')
            rospy.sleep(0.2)
            done = 1
        if done == 1:
            reevaluate()
        self.newPoint = False  # For debug. Prints if the algorithm is currently calculating a new point.
        self.reset = False
        # First, check for obstacles. If an obstacle is found between the robot and it's target, reset the path.
        # If an object is not found between the robot and it's target, and the path is valid, calculate a new waypoint.
        #TODO: when there is only one point, this breaks (index 0 is out of range?)
        if len(self.waypoints) > 0:
            dx = self.odom.pose.pose.position.x - self.waypoints[0].point.x
            dy = self.odom.pose.pose.position.y - self.waypoints[0].point.y
            diff = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        else:
            diff = 0
        # if obstacleCheck():
        #     print("obstacle reset")
        #     reevaluate()
        #     self.reset = True
        # elif diff > 5:
        #     print("diff > 5 reset")
        #     print(diff)
        #     reevaluate()
        # else:
        #print("waypointCount")
        #print(len(self.waypoints))
        if len(self.waypoints) < self.waypoint_count:
            createNewWaypoint()
        if diff < self.satisDist:
            # when close enough to waypoint 0
            # adjust the next waypoint
            if len(self.waypoints) > 1:
                #print([str(self.waypoints[i]) for i in range(0,len(self.waypoints))])
                adjustment = self.waypoints[1].auto_adjust(self.mapActual)
                # do the adjustment on all the other waypoints after
                for i in range(2,min(self.waypoint_count,len(self.waypoints))):
                    self.waypoints[i].quick_adjust(adjustment)
            # shift all waypoints so the first one goes away.
            shiftWaypoints()
            # create a new waypoint after adjustment
            if len(self.waypoints) >= self.waypoint_count:
                # if we are in explore mode, make a new waypoint at the end
                createNewWaypoint()

            self.newPoint = True
            if self.newPoint == True and diff > 0.3:
                self.newPoint = False 
        #print("last:"+str(self.last_backup)+"\nnow:"+str(self.backup))
        if self.last_backup == True and self.backup == False:
            print("backup reset")
            reevaluate()
        self.last_backup = self.backup
        
        publishWaypoints()
        # ROS info for debugging. Prints the waypoints and boolean information regarding the algorithm's status.
        #rospy.loginfo("\nRobot:" +  
        #              "\nReset        : " + str(self.reset) + 
        #              "\nCalculate    : " + str(newPoint) + 
        #              "\nFails        : " + str(self.fails) + 
        #              "\nEntropy Array: " + str(self.entropyVector))

        # Publish the waypoints to rqt for other scripts to use.


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)

