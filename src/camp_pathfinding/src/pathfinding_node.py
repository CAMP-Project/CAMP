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
import tf2_ros
import roslib
import numpy as np;
import rospy
import tf2_geometry_msgs
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point, TransformStamped, PoseStamped, Transform
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import Imu, LaserScan
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64, Int8
from camp_pathfinding.msg import Waypoints
from tf2_msgs.msg import TFMessage

class Pathfinding_Node:

    def __init__(self): 

        rospy.init_node('pathfinding', anonymous = False)


        # Introduce tf package.
        self.tfBuffer = tf2_ros.Buffer()
        self.transformListener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribe to map metadata.
        rospy.Subscriber('/map_metadata', MapMetaData, self.updateMapDimensions)

        # Subscribe to map.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)    

        # Subscribe to robot imu.
        rospy.Subscriber('/imu', Imu, self.imuUpdate)   
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # This will publish the computed waypoint information.
        self.info_publisher = rospy.Publisher('waypointList', Waypoints, queue_size = 10)

        # Create a waypoint hashmap. Stores coordinates of waypoints.
        # [x_coordinate, y_doordinate, relative_frame]
        # relative frames:
        # 0: Stop
        # 1: Odometry
        # 2: Decawave
        self.waypoints = {1 : Point(0, 0, 0),
                          2 : Point(0, 0, 0),
                          3 : Point(0, 0, 0),
                          4 : Point(0, 0, 0)}

        self.waypoint2publish = self.waypoints.get(1)

        self.mapActual = OccupancyGrid()
        self.mapData = np.array([0])                             # Variable for map storge. 
        self.mapDimensions = MapMetaData()                # Variable for map dimensions.
        self.lidar = LaserScan()                          # Variable to access parameters of the lidar.
        self.mapOrigin = MapMetaData()                    # Stores meta data about the SLAM map.
        self.imu = Imu()

        self.obstacleDetect = False                       # Indicates whether an object is blocking the path of the robot.

    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # Update Map Metadata for map dimensions. It is unlikely that the map dimensions will change.
    def updateMapDimensions(self, data):
        self.mapDimensions = data

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.mapActual = data
        self.mapData = np.array(data.data).reshape((self.mapActual.info.height, self.mapActual.info.width)) # ** This might not be necessary.

    # Update IMU data.
    def imuUpdate(self, data):
        self.imu = data

    # This method will update lidar data when new data will be available. This method grabs every parameter 
    # from the lidar node.
    def updateLidarScan(self, data):
        self.lidar = data

    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the Pathfinding algorithm
    #--------------------------------------------------------------------------------------------------------------
    def main(self):        
    
        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():
            print("This is temporary!")

        # Method for obtaining the robot's position as a distance, in meters, relative to the SLAM-generated map.
        def getRoboMapPosition():
            # Create a stamped transform.
            transform = TransformStamped()
            try:
                # Attempt to get the transform. Since the origin of the map is (0, 0), the transform will equal the translational
                # position of the robot.
                transform = self.tfBuffer.lookup_transform(self.mapActual.header.frame_id, self.imu.header.frame_id, rospy.Time(), rospy.Duration(1.0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.InvalidArgumentException):
                # Catch errors and try again.
                rospy.sleep(0.1)

            #print(transform.transform.translation) # Print for debug.
            return transform.transform.translation

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
            waypointList = Waypoints()
            waypointList.waypoint1 = self.waypoints.get(1)
            waypointList.waypoint2 = self.waypoints.get(2)
            waypointList.waypoint3 = self.waypoints.get(3)
            waypointList.waypoint4 = self.waypoints.get(4)
            self.info_publisher.publish(waypointList)

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
            
            # Calculate entropy sums.
            for k in range(1, 20):
                # Check down-left.
                if N > (y_pos - k) > 0 & N > (x_pos - k) > 0:
                    entropyDirections[0] = entropyDirections[0] + map(x_pos - k, y_pos - k)
                else:
                    entropyDirections[0] = entropyDirections[0] + 20
                
                # Check down.
                if N > (y_pos - k) > 0:
                    entropyDirections[1] = entropyDirections[1] + map(x_pos, y_pos - k)
                else:
                    entropyDirections[1] = entropyDirections[1] + 20

                # Check down-right.
                if N > (y_pos - k) > 0 & N > (x_pos + k) > 0:
                    entropyDirections[2] = entropyDirections[2] + map(x_pos + k, y_pos - k)
                else:
                    entropyDirections[2] = entropyDirections[2] + 20

                # Check right.
                if N > (x_pos + k) > 0:
                    entropyDirections[3] = entropyDirections[3] + map(x_pos + k, y_pos)
                else:
                    entropyDirections[3] = entropyDirections[3] + 20

                # Check up-right.
                if N > (y_pos + k) > 0 & N > (x_pos + k) > 0:
                    entropyDirections[4] = entropyDirections[4] + map(x_pos + k, y_pos + k)
                else:
                    entropyDirections[4] = entropyDirections[4] + 20

                # Check up.
                if N > (y_pos + k) > 0:
                    entropyDirections[5] = entropyDirections[5] + map(x_pos, y_pos + k)
                else:
                    entropyDirections[5] = entropyDirections[5] + 20

                # Check up-left.
                if N > (y_pos + k) > 0 & N > (x_pos - k) > 0:
                    entropyDirections[6] = entropyDirections[6] + map(x_pos - k, y_pos + k)
                else:
                    entropyDirections[6] = entropyDirections[6] + 20

                # Check left.
                if N > (x_pos - k) > 0:
                    entropyDirections[7] = entropyDirections[7] + map(x_pos - k, y_pos)
                else:
                    entropyDirections[7] = entropyDirections[7] + 20
                
            # Find the direction of minimum entropy.
            direction = entropyDirections.index(min(entropyDirections))

            # Select the path from the waypointMap above based on the calculated direction.
            path = waypointMap.get(direction) # Print for debug.

            # Establish path as a series of new waypoints.
            for i in range(4):
                self.waypoints[i + 1] = Point(path[0][i], path[1][i], 1)
        
        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            p = self.mapActual.data[x + (self.mapActual.info.width * y)]
            return ((-1 * p * np.log(p)) - ((1 - p) * np.log(1 - p)))
        
        resetWaypoints()

if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()
        rospy.sleep(0.1)

