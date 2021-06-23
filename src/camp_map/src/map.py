#!/usr/bin/env python
import math
import roslib
import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Vector3, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

class Camp_Map:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('camp_map', anonymous = False)   
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to odometry.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # This will publish the computed map information.
        self.map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size = 10)

        # Initialize important data types. For this script, we need access to the OccupancyGrid produced
        # by SLAM. We need the Lidar for obstacle detection. We need the odometry for positional data. 
        self.map = OccupancyGrid() 
        self.lidar = LaserScan()                         
        self.odom = Odometry()
        self.map.info.resolution = 0.05
        self.map.info.width = 100
        self.map.info.height = 100
        data = []
        data = [50 for i in range(0,self.map.info.width * self.map.info.width)]
        self.map.data = data


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

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
        def probably_free(dist,angle):
            p_m0_g0 = 0.6 #0.9 
            p_m1_g0 = 1-p_m0_g0 
            p_m1_g1 = 0.6 #0.9 
            p_m0_g1 = 1-p_m1_g1    
            # placeholder where the robot is always centered on the map.
            x = round(dist/self.map.info.resolution*math.cos(angle) + self.map.info.width/2)
            y = round(dist/self.map.info.resolution*math.sin(angle) + self.map.info.height/2)

            if ((x >= self.map.info.width) or (y >= self.map.info.width)):
                return

            index = int(x + self.map.info.width * y)
            rospy.loginfo("f x:"+str(x)+" y:"+str(y)+" i:"+str(index))
            prior = self.map.data[index] / 100.0
            post = prior*p_m0_g1/(p_m0_g0*(1-prior)+p_m0_g1*prior)
            self.map.data[index] = post * 100

        def probably_wall(dist,angle):
            p_m0_g0 = 0.6 #0.9 
            p_m1_g0 = 1-p_m0_g0 
            p_m1_g1 = 0.6 #0.9 
            p_m0_g1 = 1-p_m1_g1       
            # placeholder where the robot is always centered on the map.
            x = round(dist/self.map.info.resolution*math.cos(angle) + self.map.info.width/2)
            y = round(dist/self.map.info.resolution*math.sin(angle) + self.map.info.height/2)

            if ((x >= self.map.info.width) or (y >= self.map.info.width)):
                return

            index = int(x + self.map.info.width * y)
            rospy.loginfo("w x:"+str(x)+" y:"+str(y)+" i:"+str(index))
            prior = self.map.data[index]/100.0
            post = prior*p_m1_g1/(p_m1_g0*(1-prior)+p_m1_g1*prior)
            self.map.data[index] = post * 100


        def update_map(dist,angle):
            d = 0
            while d < dist:
                probably_free(d,angle)
                d = d + self.map.info.resolution
            probably_wall(d,angle)


        angle = self.lidar.angle_min
        for r in self.lidar.ranges:
            update_map(r,angle)
            angle = angle + self.lidar.angle_increment
        # update_map(1,0)
        self.map_publisher.publish(self.map)


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Map()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)