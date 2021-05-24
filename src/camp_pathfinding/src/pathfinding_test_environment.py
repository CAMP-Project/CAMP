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
from geometry_msgs import msg
import roslib
import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Vector3, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from camp_goto.msg import Cmd


class Pathfinding_Node:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('pathfinding', anonymous = False)

        # Subscribe to map.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)      
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to odometry.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # This will publish the computed waypoint information.
        self.point_publisher = rospy.Publisher('go_cmd', Cmd, queue_size = 10)
        
        # Instantiate publlishers for displaying the waypoints in rviz. This will be invaluable for debugging.
        self.viz_publisher_1 = rospy.Publisher('point_viz_1', PointStamped, queue_size = 10)
        self.viz_publisher_2 = rospy.Publisher('point_viz_2', PointStamped, queue_size = 10)
        self.viz_publisher_3 = rospy.Publisher('point_viz_3', PointStamped, queue_size = 10)
        self.viz_publisher_4 = rospy.Publisher('point_viz_4', PointStamped, queue_size = 10)
        self.robot_publisher = rospy.Publisher('robot_publisher', PointStamped, queue_size = 10)
        

        # Create a waypoint hashmap. Stores coordinates of waypoints. This will start as an 
        # arbitrary set of points away from the robot.
        # [x_coordinate, y_doordinate, relative_frame]
        # relative frames:
        # 0: Stop
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

        self.createPoint = False


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


        # Method for publishing waypoints to RQT. 
        def publishWaypoints():
            # First get the waypoint, which is in units.
            waypoint = self.waypoints.get(1)

            # Calculate the waypoint in meters. Cannot use getPointInMeters method as it returns
            # a PointStamped. We want this published in a format that camp_goto to read: a Point.
            x = (0.05 * waypoint.x) + self.mapActual.info.origin.position.x
            y = (0.05 * waypoint.y) + self.mapActual.info.origin.position.y
            goTo = Point(x, y, waypoint.z)

            command = Cmd()
            command.destination = goTo
            command.stop = False
            command.is_relative = False
            command.is_deca = False

            command.speed = 0.75

            command.destination_stop_distance = 0
            command.emergency_stop_distance = 0.15
            command.emergency_stop_angle = 30

            self.point_publisher.publish(command)

            # Publish the points in rviz.
            self.viz_publisher_1.publish(getPointInMeters(1))
            self.viz_publisher_2.publish(getPointInMeters(2))
            self.viz_publisher_3.publish(getPointInMeters(3))
            self.viz_publisher_4.publish(getPointInMeters(4))

            roboPosX = getRoboMapPosition().x
            roboPosY = getRoboMapPosition().y
            roboPos = PointStamped()
            roboPos.header.stamp = rospy.Time()
            roboPos.header.frame_id = "odom"
            roboPos.point = Point(roboPosX, roboPosY, 1)

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
            result_viz.header.frame_id = "odom"
            return result_viz


        # This method resets the waypoints in the event of an obstacle preventing the traversal
        # to waypoint 1 or if all paths fail around waypoint 3.
        def resetWaypoints():
            # Get position of robot as a matrix value in the map.
            #x_pos = int(getMatrixPosition()[0])
            #y_pos = int(getMatrixPosition()[1])

            x_pos = getMatrixPosition()[0]
            y_pos = getMatrixPosition()[1]

            
            # Entropy positions: [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]
            N = self.mapActual.info.height

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

            rayLimit = 40
            if x_pos > 0 and y_pos > 0:
                # Calculate entropy sums.
                for k in range(1, rayLimit):
                    # Check down-left.
                    if (y_pos - k) > 0 and (x_pos - k) > 0:
                        entropyDirections[0] = entropyDirections[0] + map(x_pos - k, y_pos - k)
                    else:
                        entropyDirections[0] = entropyDirections[0] + 20
                    
                    # Check down.
                    if (y_pos - k) > 0:
                        entropyDirections[1] = entropyDirections[1] + map(x_pos, y_pos - k)
                    else:
                        entropyDirections[1] = entropyDirections[1] + 20

                    # Check down-right.
                    if (y_pos - k) > 0 and (x_pos + k) > 0:
                        entropyDirections[2] = entropyDirections[2] + map(x_pos + k, y_pos - k)
                    else:
                        entropyDirections[2] = entropyDirections[2] + 20

                    # Check right.
                    if (x_pos + k) > 0:
                        entropyDirections[3] = entropyDirections[3] + map(x_pos + k, y_pos)
                    else:
                        entropyDirections[3] = entropyDirections[3] + 20

                    # Check up-right.
                    if (y_pos + k) > 0 and (x_pos + k) > 0:
                        entropyDirections[4] = entropyDirections[4] + map(x_pos + k, y_pos + k)
                    else:
                        entropyDirections[4] = entropyDirections[4] + 20

                    # Check up.
                    if (y_pos + k) > 0:
                        entropyDirections[5] = entropyDirections[5] + map(x_pos, y_pos + k)
                    else:
                        entropyDirections[5] = entropyDirections[5] + 20

                    # Check up-left.
                    if (y_pos + k) > 0 and (x_pos - k) > 0:
                        entropyDirections[6] = entropyDirections[6] + map(x_pos - k, y_pos + k)
                    else:
                        entropyDirections[6] = entropyDirections[6] + 20

                    # Check left.
                    if (x_pos - k) > 0:
                        entropyDirections[7] = entropyDirections[7] + map(x_pos - k, y_pos)
                    else:
                        entropyDirections[7] = entropyDirections[7] + 20
                
            # Find the direction of minimum entropy.
            direction = entropyDirections.index(min(entropyDirections))

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

            # Map limit.
            N = self.mapActual.info.height

            # Entropy positions [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]

            # The amount of squares to advance.
            d = 6

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

            # Calculate entropy squares in 8 different directions around the robot. 
            entropyDirections[0] = grabEntropySquare(point_x - 15, point_x - 5, point_y - 15, point_y - 5)
            entropyDirections[1] = grabEntropySquare(point_x - 5, point_x + 5, point_y - 15, point_y - 5)
            entropyDirections[2] = grabEntropySquare(point_x + 5, point_x + 15, point_y - 15, point_y - 5)
            entropyDirections[3] = grabEntropySquare(point_x + 5, point_x + 15, point_y - 5, point_y + 5)
            entropyDirections[4] = grabEntropySquare(point_x + 5, point_x + 15, point_y + 5, point_y + 15)
            entropyDirections[5] = grabEntropySquare(point_x - 5, point_x + 5, point_y + 5, point_y + 15)
            entropyDirections[6] = grabEntropySquare(point_x - 15, point_x - 5, point_y + 5, point_y + 15)
            entropyDirections[7] = grabEntropySquare(point_x - 15, point_x - 5, point_y - 5, point_y + 5)

            # Initialize a parameter to check if there is an obstacle between the 3rd waypoint and the generated waypoint.
            # It is assumed to be true that there is an obstacle between the points.
            isObstacle = 1

            while isObstacle == 1:
                inc = 0
                #test = 0
                for x in entropyDirections:
                    rospy.loginfo(str(inc)+": " +str(x))
                    inc = inc + 1

                # Find the direction to place a new waypoint. The square with the highest entropy is chosen.
                # The entropy tells the robot where the "highest reward" is.
                direction = entropyDirections.index(max(entropyDirections))   

                rospy.loginfo("try:"+str(direction))           

                # Get the number of squares to increase in either direction depending on the calculated direction.
                differential = advancementMap.get(direction)
                dx = differential[0]
                dy = differential[1]

                # Get the 3rd waypoint for ease of calculations.
                end = self.waypoints.get(3)

                # Check for duplicate points.
                duplicates = 0
                for i in range(1, 3):
                    if end.x + dx == self.waypoints.get(i).x and end.y + dy == self.waypoints.get(i).y:
                        duplicates = duplicates + 1

                # Connect the 3rd point and the theoretical last point.
                yMin = min([end.y, end.y + dy])
                yMax = max([end.y, end.y + dy])
                xMin = min([end.x, end.x + dx])
                xMax = max([end.x, end.x + dx])

                # Check for bounds errors or if any duplicates exist. If so, restart the sweek by setting the entropy
                # sum in that direction to 0. This will prevent that direction from being searched again since the 
                # algorithm checks for the maximum entropy value.
                if yMin < 2 or yMax > N - 1 or xMin < 2 or xMax > N - 1 or duplicates > 0:
                    if duplicates > 0:
                        rospy.loginfo("duplicates > 0 (" + str(duplicates) + ")")
                    entropyDirections[direction] = 0
                else:
                    maximum = 0
                    for i in range(xMin - 1, xMax + 1):
                        for j in range(yMin - 1, yMax + 1):
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
                

        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            # If the value at a given index is -1, return 100. This is to keep the robot from travrsing
            # to regions that have not been explored.
            if self.mapActual.data[x + (self.mapActual.info.width * y)] < 0:
                return 50
            # Return.
            else:
                return self.mapActual.data[x + (self.mapActual.info.width * y)]

        # Method to calculate the entropy at a given map index.
        def entropy(x, y):
            # First grab probability. Divide by 102 such that 100 becomes approximately 0.99.
            p = self.mapActual.data[x + (self.mapActual.info.width * y)]
            
            # If the value of the probability at the given index is negative, replace it with 0.5.
            # Note: this does not replace the value of the probability value in the OccupancyMap.
            if p < 0:
                p = 0.5
            
            p = p / 102

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
                if closestFrontObject < 0.2:
                    return True
                else:
                    return False
            else:
                return True
        
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
        
        publishWaypoints()
        robot = getMatrixPosition()
        # ROS info for debugging. Prints the waypoints and boolean information regarding the algorithm's status.
        rospy.loginfo("\nPoint 1 :\n" +
                      str(self.waypoints.get(1)) +
                      "\nPoint 2 :\n" +
                      str(self.waypoints.get(2)) +
                      "\nPoint 3 :\n" +
                      str(self.waypoints.get(3)) +
                      "\nPoint 4 :\n" +
                      str(self.waypoints.get(4)) +
                      "\nRobot   :" +
                      "\nx: " + str(robot[0]) +
                      "\ny: " + str(robot[1]) +  
                      "\nReset     : " + str(self.reset) + 
                      "\nCalculate : " + str(newPoint) + 
                      "\nFails     : " + str(self.fails))

        # Publish the waypoints to rqt for other scripts to use.


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)

