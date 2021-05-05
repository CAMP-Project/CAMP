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
import tf2_ros
import roslib
import numpy as np;
import rospy
import tf2_geometry_msgs
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point, TransformStamped, PoseStamped, Transform, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import Imu, LaserScan
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64, Int8
from camp_pathfinding.msg import Waypoints
from tf2_msgs.msg import TFMessage

class Pathfinding_Node:

    def __init__(self): 

        rospy.init_node('pathfinding', anonymous = False)
        self.rate = rospy.Rate(10)


        # Introduce tf package.
        self.tfBuffer = tf2_ros.Buffer()
        self.transformListener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribe to map.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)    

        # Subscribe to robot imu.
        rospy.Subscriber('/imu', Imu, self.imuUpdate)   
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to odometry.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # This will publish the computed waypoint information.
        self.point_publisher = rospy.Publisher('go_pos', Point, queue_size = 10)

        self.viz_publisher_1 = rospy.Publisher('point_viz_1', PointStamped, queue_size = 10)
        self.viz_publisher_2 = rospy.Publisher('point_viz_2', PointStamped, queue_size = 10)
        self.viz_publisher_3 = rospy.Publisher('point_viz_3', PointStamped, queue_size = 10)
        self.viz_publisher_4 = rospy.Publisher('point_viz_4', PointStamped, queue_size = 10)

        # Create a waypoint hashmap. Stores coordinates of waypoints.
        # [x_coordinate, y_doordinate, relative_frame]
        # relative frames:
        # 0: Stop
        # 1: Odometry
        # 2: Decawave
        self.waypoints = {1 : Point(203, 203, 1),
                          2 : Point(206, 206, 1),
                          3 : Point(209, 209, 1),
                          4 : Point(212, 212, 1)}

        self.mapActual = OccupancyGrid() 
        self.lidar = LaserScan()                          # Variable to access parameters of the lidar.
        self.imu = Imu()
        self.odom = Odometry()

        self.obstacleDetect = False                       # Indicates whether an object is blocking the path of the robot.

    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.mapActual = data

    # Update IMU data.
    def imuUpdate(self, data):
        self.imu = data

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
            result.z = 1
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
            position_in_units = np.divide(position_in_meters, 0.05)
            position_in_units = [int(round(num, 0)) for num in position_in_units]

            return position_in_units


        # Method for publishing waypoints to RQT. 
        def publishWaypoints():
            waypoint = self.waypoints.get(1)
            x = (0.05 * waypoint.x) + self.mapActual.info.origin.position.x
            y = (0.05 * waypoint.y) + self.mapActual.info.origin.position.y
            result = Point(x, y, waypoint.z)
            self.point_publisher.publish(result)

            # Publish the points in rviz.
            self.viz_publisher_1.publish(getPointInMeters(1))
            self.viz_publisher_2.publish(getPointInMeters(2))
            self.viz_publisher_3.publish(getPointInMeters(3))
            self.viz_publisher_4.publish(getPointInMeters(4))


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
            
            # Entropy positions [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]
            N = self.mapActual.info.height

            # Calculate the possible waypoint paths to take based on whichever direction is calculated.
            waypointMap = {0 : [[x_pos - 4, x_pos - 8, x_pos - 12, x_pos - 16],
                                [y_pos - 4, y_pos - 8, y_pos - 12, y_pos - 16]],
                           1 : [[x_pos, x_pos, x_pos, x_pos],
                                [y_pos - 4, y_pos - 8, y_pos - 12, y_pos - 16]],
                           2 : [[x_pos + 4, x_pos + 8, x_pos + 12, x_pos + 16],
                                [y_pos - 4, y_pos - 8, y_pos - 12, y_pos - 16]],
                           3 : [[x_pos + 4, x_pos + 8, x_pos + 12, x_pos + 16],
                                [y_pos, y_pos, y_pos, y_pos]],
                           4 : [[x_pos + 4, x_pos + 8, x_pos + 12, x_pos + 16],
                                [y_pos + 4, y_pos + 8, y_pos + 12, y_pos + 16]],
                           5 : [[x_pos, x_pos, x_pos, x_pos],
                                [y_pos + 4, y_pos + 8, y_pos + 12, y_pos + 16]],
                           6 : [[x_pos - 4, x_pos - 8, x_pos - 12, x_pos - 16],
                                [y_pos + 4, y_pos + 8, y_pos + 12, y_pos + 16]],
                           7 : [[x_pos - 4, x_pos - 8, x_pos - 12, x_pos - 16],
                                [y_pos, y_pos, y_pos, y_pos]]}        

            if x_pos > 0 and y_pos > 0:
                # Calculate entropy sums.
                for k in range(1, 20):
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
            for i in range(4):
                self.waypoints[i + 1] = Point(path[0][i], path[1][i], 1)


        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():
            # Get the position of the third waypoint.            
            point_x = self.waypoints.get(4).x
            point_y = self.waypoints.get(4).y

            # Shift the waypoint list.
            for i in range(4):
                self.waypoints[i + 1] = self.waypoints.get(i + 2)

            # Entropy positions [down-left, down, down-right, right, up-right, up, up-left, left]
            entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0]
            
            # The amount of squares to advance.
            d = 10

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

            # Calculate entropy to the upper right of waypoint 3
            entropyDirections[0] = grabEntropySquare(point_x - 15, point_x - 5, point_y - 15, point_y - 5)
            entropyDirections[1] = grabEntropySquare(point_x - 5, point_x + 5, point_y - 15, point_y - 5)
            entropyDirections[2] = grabEntropySquare(point_x + 5, point_x + 15, point_y - 15, point_y - 5)
            entropyDirections[3] = grabEntropySquare(point_x + 5, point_x + 15, point_y - 5, point_y + 5)
            entropyDirections[4] = grabEntropySquare(point_x + 5, point_x + 15, point_y + 5, point_y + 15)
            entropyDirections[5] = grabEntropySquare(point_x - 5, point_x + 5, point_y + 5, point_y + 15)
            entropyDirections[6] = grabEntropySquare(point_x - 15, point_x - 5, point_y + 5, point_y + 15)
            entropyDirections[6] = grabEntropySquare(point_x - 15, point_x - 5, point_y - 5, point_y + 5)

            # Find the direction to place a new waypoint.
            direction = entropyDirections.index(max(entropyDirections))              

            # Get the number of squares to increase in either direction depending on the calculated direction.
            differential = advancementMap.get(direction)

            self.waypoints[4] = Point(point_x + differential[0], point_y + differential[1], 1)



        def grabEntropySquare(range_x_1, range_x_2, range_y_1, range_y_2):
            result = 0
            for i in range(range_x_1, range_x_2):
                for j in range(range_y_1, range_y_2):
                    result = result + entropy(i, j)
            return result
                

        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            if self.mapActual.data[x + (self.mapActual.info.width * y)] < 0:
                return 100
            else:
                return self.mapActual.data[x + (self.mapActual.info.width * y)]

        def entropy(x, y):
            # First grab probability. Divide by 101 such that 100 becomes 0.99.
            p = self.mapActual.data[x + (self.mapActual.info.width * y)] / 102 
            
            # Return Entropy value.
            if p < 0:
                p = 0.5
            
            p = (p * 0.98) + 0.01
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
                for i in range(1, 37):
                    if ranges[i] < closestFrontObject and ranges[i] != 0:
                        closestFrontObject = self.lidar.ranges[i]
                    if ranges[360 - i] < closestFrontObject and ranges[360 - i] != 0:
                        closestFrontObject = ranges[360 - i]
                
                if closestFrontObject < 0.5:
                    return True
                else:
                    return False
            else:
                return True
        
        reset = False
        newPoint = False
        if obstacleCheck():
            resetWaypoints()
            reset = True
        else:
            dx = getMatrixPosition()[0] - self.waypoints.get(1).x
            dy = getMatrixPosition()[1] - self.waypoints.get(1).y
            if (math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))) < 3:
                createNewWaypoint()
                newPoint = True

        rospy.loginfo("\nPoint 1 :\n" +
                      str(self.waypoints.get(1)) +
                      "\nPoint 2 :\n" +
                      str(self.waypoints.get(2)) +
                      "\nPoint 3 :\n" +
                      str(self.waypoints.get(3)) +
                      "\nPoint 4 :\n" +
                      str(self.waypoints.get(4)) +
                      "\nReset     : " + str(reset) + 
                      "\nCalculate : " + str(newPoint))

        publishWaypoints()

if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()
        rospy.sleep(0.1)

