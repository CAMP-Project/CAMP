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
import numpy;
import rospy
import tf2_geometry_msg
import exceptions.
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import Imu, LaserScan
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64
from camp_pathfinding.msg import Waypoints
from tf2_msgs.msg import TFMessage

class Pathfinding_Node:

    def __init__(self): 

        rospy.init_node('pathfinding', anonymous = False)


        # Introduce tf package.
        self.tfBuffer = tf2_ros.Buffer()
        self.transformListener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribe to important nodes.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)          # Subscribe to map.

        # Subscribed to map meta data node. Could provide useful information.
        rospy.Subscriber('/map_metadata', MapMetaData, self.getMapOrigin)       # Subscribe to map meta data. 
        
        # Subscribe to robot odometry.
        rospy.Subscriber('/odom', Odometry, self.updateRobotPosition)    # Subscribe to odometry. ** Might not be necessary.
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)       # Subscribe to lidar.

        rospy.Subscriber('/tf_static', TFMessage, self.staticUpdate)  # Subscribe to static transform frames.

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

        self.baseLink = PoseStamped()
        self.botPosition = PoseStamped()                       # Variable for robot position.
        self.map = OccupancyGrid()                        # Variable for map storge. 
        self.lidar = LaserScan()                          # Variable to access parameters of the lidar.
        self.mapOrigin = MapMetaData()                    # Stores meta data about the SLAM map.

        self.obstacleDetect = False                       # Indicates whether an object is blocking the path of the robot.
        self.entropyDirections = [0, 0, 0, 0, 0, 0, 0, 0] # This will hold on to entropy data to determine where to put a new waypoint.

    # This method will update the position of the robot relative to odometry. 
    def updateRobotPosition(self, data):
        self.botPosition.pose = data.pose.pose
        self.botPosition.header.frame_id = 'odometry'

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.map = data

    # Update static frame information for robot's base_link.
    def staticUpdate(self, data):
        self.baseLink = data.transforms[3]

    # This method will call the meta data from the map.
    def getMapOrigin(self, data):
        self.mapOrigin = data

    # This method will update lidar data when new data will be available. This method grabs every parameter 
    # from the lidar node.
    def updateLidarScan(self, data):
        self.lidar = data


    # This method will read in LiDAR data to determine if the current path needs to be reset.
    #**************************************************************************************
    # This method needs to read lidar values at a specified angle. I don't know
    # exactly how the lidar data appears in an array nor which values need to be changed
    #**************************************************************************************
    def main(self):

        # This method will reset the waypoint list.
        #**************************************************************************************
        # This method will depend entirely on the angle at which the robot is facing, which
        # needs to be read relative to the map matrix.
        #**************************************************************************************
        def resetWaypoints():
            for point in self.waypoints.keys():
                self.waypoints[point] = [0, 0]
    
        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():
            print("This is temporary!")


        def getRoboMapPosition():
            transform = None
            try:
                print(self.baseLink.header.frame_id)
                print(self.map.header.frame_id)
                transform = self.tfBuffer.lookup_transform(self.map.header.frame_id, self.baseLink.header.frame_id, rospy.Time(), rospy.Duration(1.0))
                print(transform)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ):
                rospy.sleep(0.1)

            if transform is not None:
                test = tf2_geometry_msgs.do_transform_pose(self.botPosition, transform)
                print(test.pose)   

        def publishWaypoints():
            index = 1
            waypointList = Waypoints()
            waypointList.waypoint1 = self.waypoints.get(1)
            waypointList.waypoint2 = self.waypoints.get(2)
            waypointList.waypoint3 = self.waypoints.get(3)
            waypointList.waypoint4 = self.waypoints.get(4)
            self.info_publisher.publish(waypointList)

        getRoboMapPosition()
        

if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()
        rospy.sleep(0.1)

