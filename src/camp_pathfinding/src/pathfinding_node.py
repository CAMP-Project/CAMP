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

        self.mapActual = OccupancyGrid()
        self.mapData = np.array([0])                             # Variable for map storge. 
        self.mapDimensions = MapMetaData()                # Variable for map dimensions.
        self.lidar = LaserScan()                          # Variable to access parameters of the lidar.
        self.mapOrigin = MapMetaData()                    # Stores meta data about the SLAM map.
        self.imu = Imu()

        self.obstacleDetect = False                       # Indicates whether an object is blocking the path of the robot.
        self.entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0] # This will hold on to entropy data to determine where to put a new waypoint.


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

        # This method will reset the waypoint list.
        def resetWaypoints():
            for point in self.waypoints.keys():
                self.waypoints[point] = [0, 0]
    
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

        def calculateEntropyData(x, y):
            print("This is temporary")
        
        #**********
        # DEBUG
        #**********
        print(getRoboMapPosition())
        print(getMatrixPosition())
        print(len(self.mapActual.data))
        if len(self.mapActual.data) != 0:
            print(self.mapActual.data[313+ (self.mapActual.info.width * 163)])

if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()
        rospy.sleep(0.1)

