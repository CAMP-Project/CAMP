#!/usr/bin/env python
""""
----------------------------------------------------------------------------------------
Original Author : Sean Carda
Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
Original Date   : 4/15/2021
Description     : This script implements a multiagent version of pathfinding utilizing
                  the multimaster architecture.

Change Log:

8/9/2021 - Sean Carda: Initial script creation.

----------------------------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------------------
# Standard python imports.
import math
import roslib
import rospy
roslib.load_manifest('rospy')
#------------------------------------------------------------------------------------------------
# Standard ROS message imports.
from geometry_msgs.msg import Vector3, Point, PointStamped, TransformStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
#------------------------------------------------------------------------------------------------
# Custom message imports.
from camp_goto.msg import Cmd
from brian.msg import DecawaveLabeled, DecawaveMapLabeled, OccupancyMapLabeled, OdometryLabeled
from brian.msg import RobotKeyList
#------------------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------------------


class Pathfinding_Multi:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('pathfinding', anonymous = False)

        # Subscribe to the map topic.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)      
        
        # Subscribe to the LiDAR topic.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to the odometry topic.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # Subscribe to the backup state topic.
        rospy.Subscriber('/backup_state', Bool, self.updateBackup)

        # This will publish the computed waypoint information.
        self.point_publisher = rospy.Publisher('go_cmd', Cmd, queue_size = 10)
        
        # Instantiate publlishers for displaying the waypoints in rviz. This is valuable for debugging.
        self.viz_publisher_1 = rospy.Publisher('point_viz_1', PointStamped, queue_size = 10)
        self.viz_publisher_2 = rospy.Publisher('point_viz_2', PointStamped, queue_size = 10)
        self.viz_publisher_3 = rospy.Publisher('point_viz_3', PointStamped, queue_size = 10)
        self.viz_publisher_4 = rospy.Publisher('point_viz_4', PointStamped, queue_size = 10)

        # The region publisher publishes the point in which the robot is currently tring to plan 
        # a new point. The region point is located in the center of an entropy grid.
        self.region_publisher = rospy.Publisher('region', PointStamped, queue_size = 10)

        # This publishes the position of the robot.
        self.robot_publisher = rospy.Publisher('robot_publisher', PointStamped, queue_size = 10)
        

        # Create a waypoint hashmap. Stores coordinates of waypoints. This will start as an 
        # arbitrary set of points away from the robot.
        # Point(x_coordinate, y_doordinate, relative_frame)
        # relative frames:
        # 0: Stop Moving
        # 1: Odometry
        # 2: Decawave
        self.waypoints = {1 : Point(206, 206, 1),
                          2 : Point(212, 212, 1),
                          3 : Point(218, 218, 1),
                          4 : Point(224, 224, 1)}

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
        self.satisDist = 1
        
        # Tracks whether the robot is trying to create a point.
        self.createPoint = False

        # Tracks the values stored in the entropy array during a reset calculation.
        self.entropyVector = [0, 0, 0, 0, 0, 0, 0, 0]
        
        # Backup checks.
        self.backupOld = False
        self.backupNew = False

        #---------------------------------------------------------------------------------
        # NEW INIT CODE FOR TF.
        
        # Instantiate a new tf buffer and a new tf listener.
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #---------------------------------------------------------------------------------
        # NEW INIT CODE FOR MULTIMASTER.

        # Create a list object to hold on to the different hosts on the network.
        self.hostList = []

        # Add map variables in which to store incoming data.
        self.odomMap = {} # Odom-based map storage.
        self.odomPos = {} # Odom-based positional storage.
        self.decaMap = {} # Deca-based map storage.
        self.decaPos = {} # Deca-based positional storage.

        # Stores subscriber objects which may be deleted later.
        self.odomPosSubs = {} # Odom-based positional subscriber storage.
        self.odomMapSubs = {} # Odom-based map subscriber storage.
        self.decaPosSubs = {} # Deca-based positional subscriber storage.
        self.decaMapSubs = {} # Deca-based map subscriber storage.

        # Subscribe to the list of other roscores on the current network.
        rospy.Subscriber('/host_list', RobotKeyList, self.update_list)
        rospy.sleep(1)


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


    # Callback to update the most recent backup state.
    def updateBackup(self, data):
        self.backupNew = data


    # Callback to update the list of hosts on the current network.
    def update_list(self, data):
        globalKeys = data.robotKeys
        
        # First, check for keys contained in the local host list that are NOT in the current global list.
        for key in self.hostList:
            if key not in globalKeys:
                # Remove the entry from the local host list.
                self.hostList.pop(self.hostList.index(key))

                # Remove the entry from the odom map storage variables.
                self.odomMap.pop(key)
                self.odomMapSubs.pop(key)

                # Remove the entry from the odom pos storage variables.
                self.odomPos.pop(key)
                self.odomPosSubs.pop(key)

                # Remove the entry from the deca map storage variables.
                self.decaMap.pop(key)
                self.decaMapSubs.pop(key)

                # Remove the entry from the deca pos storage variables.
                self.decaPos.pop(key)
                self.decaPosSubs.pop(key)

        # Then, check for keys contained in the global list that are NOT in the local list.
        for key in globalKeys:
            if key not in self.hostList:
                self.hostList.append(key)
                self.odomMap[key] = None
                self.odomPos[key] = None
                self.decaMap[key] = None
                self.decaPos[key] = None

                # Create a new odometry subscriber for the new host.
                odomPosLabel = "/" + key + "/odom_labeled"
                odomMapLabel = "/" + key + "/odom_map_labeled"
                decaPosLabel = "/" + key + "/deca_labeled"
                decaMapLabel = "/" + key + "/deca_map_labeled"
                self.odomPosSubs[key] = rospy.Subscriber(odomPosLabel, OdometryLabeled, self.odomPosUpdate)
                self.odomMapSubs[key] = rospy.Subscriber(odomMapLabel, OccupancyMapLabeled, self.odomMapUpdate)
                self.decaPosSubs[key] = rospy.Subscriber(decaPosLabel, DecawaveLabeled, self.decaPosUpdate)
                self.decaMapSubs[key] = rospy.Subscriber(decaMapLabel, DecawaveMapLabeled, self.decaMapUpdate)

    
    # Callback method for obtaining the odometry positions of all the robots on the network.
    # NOTE: this may become obsolete as I do not think we will be using the odometries of other
    # robots to implement multiagent pathfinding.
    def odomPosUpdate(self, data):
        self.odomPos[data.name] = data.odom

    
    # Callback method for obtaining all of the odometry maps of all the robots on the network.
    def odomMapUpdate(self, data):
        self.odomMap[data.name] = data.map


    # Callback method for obtaining the decawave positions of all the robots on the network.
    def decaPosUpdate(self, data):
        self.decaPos[data.name] = data.pose

    # ***************************************************************************************************************
    # NOTE: I am undure whether or not this needs to be created in this script, or if we should just pull from the
    # combined deca map. This method would grab all the deca-transformed maps from other robots, which I'm not
    # sure is what we want.
    # ***************************************************************************************************************
    def decaMapUpdate(self, data):
        self.decaMap[data.name] = data.pose


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
            
            # The z position in this context, as described above, is the frame in which the robot will move. The value
            # here will be 1 since we want the robot to move relative to its own odometry.
            result.z = 1

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
            for i in range(2):
                position_in_units[i] = round(position_in_meters[i] / 0.05, 0)
                position_in_units[i] = int(position_in_units[i])

            # Return.
            return position_in_units


        # Method for publishing waypoints to RQT and RVIZ. 
        def publishWaypoints():
            # First get the waypoint, which is in units.
            waypoint = self.waypoints.get(1)

            # Calculate the waypoint in meters. Cannot use getPointInMeters method as it returns
            # a PointStamped. We want this published in a format that camp_goto to read: a Point.
            x = (0.05 * waypoint.x) + self.mapActual.info.origin.position.x
            y = (0.05 * waypoint.y) + self.mapActual.info.origin.position.y
            goTo = Point(x, y, waypoint.z)

            # Create all of the information necessary for the Cmd publisher.
            command = Cmd()
            command.destination = goTo
            command.stop = False
            command.is_relative = False
            command.is_deca = False
            command.speed = 0.43
            command.destination_stop_distance = 0
            command.emergency_stop_distance = 0.15
            command.emergency_stop_angle = 30

            # Publish the command to the controller.
            self.point_publisher.publish(command)

            # Publish the points in rviz.
            self.viz_publisher_1.publish(getPointInMeters(1))
            self.viz_publisher_2.publish(getPointInMeters(2))
            self.viz_publisher_3.publish(getPointInMeters(3))
            self.viz_publisher_4.publish(getPointInMeters(4))

            # Prepare the information necessary for the robot position publisher.
            roboPosX = getRoboMapPosition().x
            roboPosY = getRoboMapPosition().y
            roboPos = PointStamped()
            roboPos.header.stamp = rospy.Time()
            roboPos.header.frame_id = "map"
            roboPos.point = Point(roboPosX, roboPosY, 1)

            # Publish the robot's position.
            self.robot_publisher.publish(roboPos)


        # Submethod for obtaining the pointstamp necessary for rviz plotting.
        def getPointInMeters(point):
            # Get the waypoint in units.
            waypoint = self.waypoints.get(point)
            x = (0.05 * waypoint.x) + self.mapActual.info.origin.position.x
            y = (0.05 * waypoint.y) + self.mapActual.info.origin.position.y
            result = Point(x, y, waypoint.z)

            # Result as a PointStamp.
            result_viz = PointStamped()
            result_viz.point = result
            result_viz.header.stamp = rospy.Time()
            result_viz.header.frame_id = "map"
            return result_viz


        # This method resets the waypoints in the event of an obstacle preventing the traversal
        # to waypoint 1 or if all paths fail around waypoint 3.
        def resetWaypoints():
            # Get position of robot as a matrix value in the map.
            x_pos = getMatrixPosition()[0]
            y_pos = getMatrixPosition()[1]
            
            # Entropy positions: [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]

            # Intermediate step for automating the reset calculation process. ** Needs to be completed **
            checkSigns = [[-1,-1],[0,-1],[1,-1],[1,0],[1,1],[0,1],[-1,1],[-1,0]]

            # Checks for whether an obstacle has been found in the 8 checked directions.
            # If an object has been found in a direction, the flag is set to True. This will
            # make all indices checked after the obstacle as 9999 to strongly discourage that direction.
            foundWalls = [False, False, False, False, False, False, False, False]

            # Distances at which at space points.
            dist1 = 6
            dist2 = 2 * dist1
            dist3 = 3 * dist1
            dist4 = 4 * dist1

            # Calculate the possible waypoint paths to take based on whichever direction is calculated.
            waypointMap = {0 : [[x_pos - dist1, x_pos - dist2, x_pos - dist3, x_pos - dist4],
                                [y_pos - dist1, y_pos - dist2, y_pos - dist3, y_pos - dist4]],
                           
                           1 : [[x_pos, x_pos, x_pos, x_pos],
                                [y_pos - dist1, y_pos - dist2, y_pos - dist3, y_pos - dist4]],
                           
                           2 : [[x_pos + dist1, x_pos + dist2, x_pos + dist3, x_pos + dist4],
                                [y_pos - dist1, y_pos - dist2, y_pos - dist3, y_pos - dist4]],
                           
                           3 : [[x_pos + dist1, x_pos + dist2, x_pos + dist3, x_pos + dist4],
                                [y_pos, y_pos, y_pos, y_pos]],
                           
                           4 : [[x_pos + dist1, x_pos + dist2, x_pos + dist3, x_pos + dist4],
                                [y_pos + dist1, y_pos + dist2, y_pos + dist3, y_pos + dist4]],
                           
                           5 : [[x_pos, x_pos, x_pos, x_pos],
                                [y_pos + dist1, y_pos + dist2, y_pos + dist3, y_pos + dist4]],
                           
                           6 : [[x_pos - dist1, x_pos - dist2, x_pos - dist3, x_pos - dist4],
                                [y_pos + dist1, y_pos + dist2, y_pos + dist3, y_pos + dist4]],
                           
                           7 : [[x_pos - dist1, x_pos - dist2, x_pos - dist3, x_pos - dist4],
                                [y_pos, y_pos, y_pos, y_pos]]}        

            # Sets the limit for point checks.
            rayLimit = 40

            # Information for difference optimization.
            offset = 26
            power = 2
            if x_pos > 0 and y_pos > 0:
                # Calculate entropy sums.
                for k in range(1, rayLimit):
                    # Check down-left.
                    if (y_pos - k) > 0 and (x_pos - k) > 0 and foundWalls[0] is False:
                        if map(x_pos - k, y_pos - k) > 90:
                            foundWalls[0] = True
                        entropyDirections[0] = entropyDirections[0] + pow(map(x_pos - k, y_pos - k) - offset, power)/k
                    else:
                        entropyDirections[0] = entropyDirections[0] + 9999

                    # Check down.
                    if (y_pos - k) > 0 and foundWalls[1] is False:
                        if map(x_pos, y_pos - k) > 90:
                            foundWalls[1] = True
                        entropyDirections[1] = entropyDirections[1] + pow(map(x_pos, y_pos - k) - offset, power)/k
                    else:
                        entropyDirections[1] = entropyDirections[1] + 9999

                    # Check down-right.
                    if (y_pos - k) > 0 and (x_pos + k) > 0 and foundWalls[2] is False:
                        if map(x_pos + k, y_pos - k) > 90:
                            foundWalls[2] = True
                        entropyDirections[2] = entropyDirections[2] + pow(map(x_pos + k, y_pos - k) - offset, power)/k
                    else:
                        entropyDirections[2] = entropyDirections[2] + 9999

                    # Check right.
                    if (x_pos + k) > 0 and foundWalls[3] is False:
                        if map(x_pos + k, y_pos) > 90:
                            foundWalls[3] = True
                        entropyDirections[3] = entropyDirections[3] + pow(map(x_pos + k, y_pos) - offset, power)/k
                    else:
                        entropyDirections[3] = entropyDirections[3] + 9999

                    # Check up-right.
                    if (y_pos + k) > 0 and (x_pos + k) > 0 and foundWalls[4] is False:
                        if map(x_pos + k, y_pos + k) > 90:
                            foundWalls[4] = True
                        entropyDirections[4] = entropyDirections[4] + pow(map(x_pos + k, y_pos + k) - offset, power)/k
                    else:
                        entropyDirections[4] = entropyDirections[4] + 9999

                    # Check up.
                    if (y_pos + k) > 0 and foundWalls[5] is False:
                        if map(x_pos, y_pos + k) > 90:
                            foundWalls[5] = True
                        entropyDirections[5] = entropyDirections[5] + pow(map(x_pos, y_pos + k) - offset, power)/k
                    else:
                        entropyDirections[5] = entropyDirections[5] + 9999

                    # Check up-left.
                    if (y_pos + k) > 0 and (x_pos - k) > 0 and foundWalls[6] is False:
                        if map(x_pos - k, y_pos + k) > 90:
                            foundWalls[6] = True
                        entropyDirections[6] = entropyDirections[6] + pow(map(x_pos - k, y_pos + k) - offset, power)/k
                    else:
                        entropyDirections[6] = entropyDirections[6] + 9999

                    # Check left.
                    if (x_pos - k) > 0 and foundWalls[7] is False:
                        if map(x_pos - k, y_pos) > 90:
                            foundWalls[7] = True
                        entropyDirections[7] = entropyDirections[7] + pow(map(x_pos - k, y_pos) - offset, power)/k
                    else:
                        entropyDirections[7] = entropyDirections[7] + 9999
                
            # Find the direction of minimum entropy.
            direction = entropyDirections.index(min(entropyDirections))
 
            # Set the entropyVector debug parameter.
            self.entropyVector = entropyDirections

            # Select the path from the waypointMap above based on the calculated direction.
            path = waypointMap.get(direction)

            # Establish path as a series of new waypoints.
            self.waypoints[1] = Point(path[0][0], path[1][0], 1)
            self.waypoints[2] = Point(path[0][1], path[1][1], 1)
            self.waypoints[3] = Point(path[0][2], path[1][2], 1)
            self.waypoints[4] = Point(path[0][3], path[1][3], 1)


        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():

            # Get the position of the third waypoint.            
            point_x = self.waypoints.get(4).x
            point_y = self.waypoints.get(4).y

            # Entropy positions [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]

            # The amount of squares to advance.
            d = 8

            # Traversal movements in each direction. Premade depending on the direction to be calculated.
            advancementMap = {
                0 : [-d, -d],
                1 : [0, -d],
                2 : [d, -d],
                3 : [d, 0],
                4 : [d, d],
                5 : [0, d],
                6 : [-d, d],
                7 : [-d, 0] 
            }

            # Parameters for larger entropy grid calculations.
            limitMin = 5
            limitMax = 15
            outerEdge = 30

            # Calculate entropy squares in 8 different directions around the robot. 
            entropyDirections[0] = grabEntropySquare(point_x - limitMax, point_x - limitMin, point_y -  limitMax, point_y - limitMin) + 0.1*grabEntropySquare(point_x - limitMax - outerEdge, point_x -  limitMax, point_y - limitMax - outerEdge, point_y - limitMax)/9
            entropyDirections[1] = grabEntropySquare(point_x - limitMin, point_x + limitMin, point_y -  limitMax, point_y - limitMin) + 0.1*grabEntropySquare(point_x - limitMax, point_x + limitMax, point_y - limitMax - outerEdge, point_y - limitMax)/9
            entropyDirections[2] = grabEntropySquare(point_x + limitMin, point_x +  limitMax, point_y -  limitMax, point_y - limitMin) + 0.1*grabEntropySquare(point_x +  limitMax, point_x + limitMax + outerEdge, point_y - limitMax - outerEdge, point_y - limitMax)/9
            entropyDirections[3] = grabEntropySquare(point_x + limitMin, point_x +  limitMax, point_y - limitMin, point_y + limitMin) + 0.1*grabEntropySquare(point_x +  limitMax, point_x + limitMax + outerEdge, point_y - limitMax, point_y + limitMax)/9
            entropyDirections[4] = grabEntropySquare(point_x + limitMin, point_x +  limitMax, point_y + limitMin, point_y + limitMax) + 0.1*grabEntropySquare(point_x +  limitMax, point_x + limitMax + outerEdge, point_y +  limitMax, point_y + limitMax + outerEdge)/9
            entropyDirections[5] = grabEntropySquare(point_x - limitMin, point_x + limitMin, point_y + limitMin, point_y + limitMax) + 0.1*grabEntropySquare(point_x - limitMax, point_x + limitMax, point_y +  limitMax, point_y + limitMax + outerEdge)/9
            entropyDirections[6] = grabEntropySquare(point_x -  limitMax, point_x - limitMin, point_y + limitMin, point_y + limitMax) + 0.1*grabEntropySquare(point_x - limitMax - outerEdge, point_x -  limitMax, point_y +  limitMax, point_y + limitMax + outerEdge)/9
            entropyDirections[7] = grabEntropySquare(point_x -  limitMax, point_x - limitMin, point_y - limitMin, point_y + limitMin) + 0.1*grabEntropySquare(point_x - limitMax - outerEdge, point_x -  limitMax, point_y - limitMax, point_y + limitMax)/9

            # Initialize a parameter to check if there is an obstacle between the 3rd waypoint and the generated waypoint.
            # It is assumed to be true that there is an obstacle between the points.
            isObstacle = 1

            while isObstacle == 1:
                # Debug information.
                inc = 0
                for x in entropyDirections:
                    rospy.loginfo(str(inc)+": " +str(x))
                    inc = inc + 1

                # Find the direction to place a new waypoint. The square with the highest entropy is chosen.
                # The entropy tells the robot where the "highest reward" is.
                direction = entropyDirections.index(max(entropyDirections))   

                # Debug for current direction check.
                rospy.loginfo("try:"+str(direction))           

                # Get the number of squares to increase in either direction depending on the calculated direction.
                differential = advancementMap.get(direction)
                dx = differential[0]
                dy = differential[1]

                # Get the 3rd waypoint for ease of calculations.
                end = self.waypoints.get(4)

                # Check for duplicate points.
                duplicates = 0
                for i in range(1, 4):
                    rospy.loginfo("Point " + str(i) +"\nNew point: ("+str(end.x + dx)+","+str(end.y + dy)+")\nOld point: ("+str(self.waypoints.get(i).x)+","+str(self.waypoints.get(i).y)+")") 
                    if end.x + dx == self.waypoints.get(i).x and end.y + dy == self.waypoints.get(i).y:
                        rospy.loginfo("Match")     
                        duplicates = duplicates + 1
                    else:
                        rospy.loginfo("No Match")

                # Connect the 3rd point and the theoretical last point.
                yMin = min([end.y, end.y + dy])
                yMax = max([end.y, end.y + dy])
                xMin = min([end.x, end.x + dx])
                xMax = max([end.x, end.x + dx])

                # Check for bounds errors or if any duplicates exist. If so, restart the sweek by setting the entropy
                # sum in that direction to 0. This will prevent that direction from being searched again since the 
                # algorithm checks for the maximum entropy value.
                if duplicates > 0:
                    rospy.loginfo("duplicates > 0 (" + str(duplicates) + ")")
                    entropyDirections[direction] = 0
                else:
                    maximum = 0
                    for i in range(xMin - 1, xMax + 1):
                        for j in range(yMin - 1, yMax + 1):
                            regionX = (0.05 * round((xMax + xMin)/2)) + self.mapActual.info.origin.position.x
                            regionY = (0.05 * round((yMax + yMin)/2)) + self.mapActual.info.origin.position.y
                            regionPos = PointStamped()
                            regionPos.header.stamp = rospy.Time()
                            regionPos.header.frame_id = "map"
                            regionPos.point = Point(regionX, regionY, 1)
                            self.region_publisher.publish(regionPos)
                            if map(i, j) > maximum:
                                maximum = map(i, j)

                    if maximum > 70:
                        entropyDirections[direction] = 0
                        rospy.loginfo("maximum > 0.7 (" + str(maximum) + ")")
                    else:
                        self.waypoints[1] = self.waypoints[2]
                        self.waypoints[2] = self.waypoints[3]
                        self.waypoints[3] = self.waypoints[4]
                        self.waypoints[4] = Point(point_x + dx, point_y + dy, 1)
                        isObstacle = 0
                        self.fails = 0

                # Track number of fails.
                self.fails = self.fails + 1
                rospy.loginfo("Fails: " + str(self.fails))

                if self.fails > 7:
                    resetWaypoints()
                    self.fails = 0
                    isObstacle = 0


        # Method to calculate an entire region of entropy. Reduces the necessary lines of code to write.
        def grabEntropySquare(range_x_1, range_x_2, range_y_1, range_y_2):
            # Initialize the result of the scan.
            result = 0

            # On the bounds of the given entropy region, calculate the total entropy.
            for i in range(range_x_1, range_x_2):
                for j in range(range_y_1, range_y_2):
                    result = result + entropy(i, j)

            # Return.
            return result


        # Method to calculate a circular region of entropy. 
        def grabEntropyCircle(radius, center_x, center_y):
            
            # Establish a reasonable set of bounds in which to examine data points. We do not
            # want to have to search through the entire occupancy grid for values. Expecially
            # if the occupancy grid becomes very large.
            bound_x_left = center_x - radius
            bound_x_right = center_x + radius
            bound_y_down = center_y - radius
            bound_y_up = center_y + radius
            result = 0

            # In the bounds we created, determine if a point lies within the inscribed circle defined by
            # the given radius and center.
            for i in range(bound_x_left, bound_x_right + 1):
                for j in range(bound_y_up, bound_y_down + 1):
                    if math.sqrt(pow(i - center_x, 2) + pow(j - center_y, 2)) < radius:
                        result = result + entropy(i, j)

            # Return result.
            return result
                

        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            # If the value at a given index is -1, return 50. This is to keep the robot from travrsing
            # to regions that have not been explored.
            if len(self.mapActual.data) <= 0:
                return 0
            if self.mapActual.data[x + (self.mapActual.info.width * y)] < 0:
                return 50
            # Return.
            else:
                return self.mapActual.data[x + (self.mapActual.info.width * y)]

        # Method to calculate the entropy at a given map index.
        def entropy(x, y):
            # First grab probability. Divide by 102 such that 100 becomes approximately 0.99.
            p = self.mapActual.data[x + (self.mapActual.info.width * y)]
            #print(p)
            
            # If the value of the probability at the given index is negative, replace it with 0.5.
            # Note: this does not replace the value of the probability value in the OccupancyMap.
            if p < 0:
                p = 50
            #print(p)
            p = p / 100.0
            print(p)
            # Quick calculation to ensure that the probability is between 0.01 and 0.99.
            p = (p * 0.98) + 0.01
            #print(p)
            # Return the entropy.
            #print((-p * math.log(p, 2)) - ((1 - p) * math.log(1 - p, 2)))
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
                if closestFrontObject < 0.35:
                    return True
                else:
                    return False
            else:
                return True


        # Method for computing the transform of a pose in the source frame (the pose's original frame), to a pose in
        # the target frame (the frame you want to be in).
        def transformActions(poseToBeTransformed, target_frame, source_frame, action):
            poseToPoseTransform = TransformStamped()

            # Try performing the transform. First, determine if the pose can be transformed. Second, if the transform
            # can occur, look up the transform in the tf2 buffer. Third, once a transform has been found, use tf2
            # to calculate the resulting pose of the pose to-be-transformed subject to the looked-up transform.
            try:
                self.tfBuffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))
                poseToPoseTransform = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
                poseInNewFrame = tf2_geometry_msgs.do_transform_pose(poseToBeTransformed, poseToPoseTransform)

            # If the transform cannot occur (an exception has been raised), catch it, and sleep.
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(1)

            # Here, the user can select which object they would like returned. This prevents the need
            # to create two methods which are so similar in code. The 'action' parameter decides which
            # object is to be returned. If nothing has been entered, it will, by default, return
            # the new pose of the pose-to-be-transformed.
            if action is 'pose':
                return poseInNewFrame
            elif action is 'transform':
                return poseToPoseTransform
            else:
                return poseInNewFrame

        
        # Main functionality of the pathfinding code. 
        newPoint = False  # For debug. Prints if the algorithm is currently calculating a new point.
        self.reset = False
        # First, check for obstacles. If an obstacle is found between the robot and it's target, reset the path.
        # If an object is not found between the robot and it's target, and the path is valid, calculate a new waypoint.
        dx = getMatrixPosition()[0] - self.waypoints.get(1).x
        dy = getMatrixPosition()[1] - self.waypoints.get(1).y
        diff = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if obstacleCheck():
            resetWaypoints()
            self.reset = True
        elif diff > 100:
            resetWaypoints()
        else:
            if diff < 3 and self.createPoint == False:
                createNewWaypoint()
                self.newPoint = True
            if newPoint == True and diff > 6:
                self.newPoint = False 
            if self.backupOld is True and self.backupNew is False:
                resetWaypoints()
        
        publishWaypoints()


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Pathfinding_Multi()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)