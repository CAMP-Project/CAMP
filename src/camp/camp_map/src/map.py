#!/usr/bin/env python
import math
import roslib
#import numpy as np;
import rospy
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Pose, PoseStamped #Vector3, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

import tf2_ros as tfr
import tf2_geometry_msgs as tfgm

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
        self.map.header.frame_id = "deca"
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
        data = [50 for i in range(0,self.map.info.width * self.map.info.height)]
        self.map.data = data

        self.tf_buffer = tfr.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tfr.TransformListener(self.tf_buffer)
        self.deca_pose = PoseStamped()


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
        # build a poseStamped for transforming
        odom_pose = PoseStamped()
        odom_pose.pose =  self.odom.pose.pose
        odom_pose.header = self.odom.header

        try:
            transform = self.tf_buffer.lookup_transform('deca','odom',rospy.Time(0),rospy.Duration(1.0))
        except Exception as x:
            rospy.logwarn("transform goofed:\n%s",str(x))
        self.deca_pose = tfgm.do_transform_pose(odom_pose, transform)


    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the mapping algorithm
    #--------------------------------------------------------------------------------------------------------------
    def main(self):
        def expand_map(direction):
            expand_amount = 50
            xbuffer = 0
            ybuffer = 0
            old_width = self.map.info.width
            new_width = old_width
            old_height = self.map.info.height
            new_height = old_height

            if direction == "+x":
                new_width = new_width + expand_amount
            if direction == "-x":
                xbuffer = expand_amount
                new_width = new_width + expand_amount
            if direction == "+y":
                new_height = new_height + expand_amount
            if direction == "-y":
                ybuffer = expand_amount
                new_height = new_height + expand_amount

            newdata = []
            newdata = [50 for i in range(0,new_width * new_height)]
            for m in range(0,old_width):
                for n in range(0,old_height):
                    old_index = m + old_width * n
                    new_index = (m + xbuffer) + new_width * (n + ybuffer)
                    newdata[new_index] = self.map.data[old_index]
            
            self.map.info.width = new_width
            self.map.info.height = new_height
            self.map.info.origin.position.x = self.map.info.origin.position.x - xbuffer * self.map.info.resolution
            self.map.info.origin.position.y = self.map.info.origin.position.y - ybuffer * self.map.info.resolution
            self.map.info.origin.position.z = 0
            
            self.map.data = newdata



        def get_robot_angle():
            #euler = tf.transformations.euler_from_quaternion(self.odom.pose.pose.orientation)
            quat = (self.deca_pose.pose.orientation.x,
                    self.deca_pose.pose.orientation.y,
                    self.deca_pose.pose.orientation.z,
                    self.deca_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quat)
            robot_angle = euler[2]
            return robot_angle
            

        def update_square(scan_dist,scan_angle,robot_angle,update_param,trust):
            p_m0_g0 = trust #0.7 
            p_m1_g0 = 1-p_m0_g0 
            p_m1_g1 = trust #0.7 
            p_m0_g1 = 1-p_m1_g1       
            # placeholder where the robot is always centered on the map.
            x = round((scan_dist*math.cos(scan_angle + robot_angle) + self.deca_pose.pose.position.x - self.map.info.origin.position.x)/self.map.info.resolution)
            y = round((scan_dist*math.sin(scan_angle + robot_angle) + self.deca_pose.pose.position.y - self.map.info.origin.position.y)/self.map.info.resolution)

            # if ((x >= self.map.info.width) or (y >= self.map.info.width) or (x < 0) or (y < 0)):
            #     return
            #print("x: " + str(x) + "y: " + str(y))
            #print(self.map.info.width)
            expand_threshold = 50
            if (x >= self.map.info.width - expand_threshold):
                expand_map("+x")
                print("expanded +x")
            if (y >= self.map.info.height - expand_threshold):
                expand_map("+y")
                print("expanded +y")
            if (x < expand_threshold):
                expand_map("-x")
                print("expanded -x")
            if (y < expand_threshold):
                expand_map("-y")
                print("expanded -y")

            index = int(x + self.map.info.width * y)
            #rospy.loginfo("w x:"+str(x)+" y:"+str(y)+" i:"+str(index))
            #print(index)
            prior = self.map.data[index]/100.0
            if update_param == "free":
                post = prior*p_m0_g1/(p_m0_g0*(1-prior)+p_m0_g1*prior)
            elif update_param == "wall":
                post = prior*p_m1_g1/(p_m1_g0*(1-prior)+p_m1_g1*prior)
            else:
                print(update_param)
                print("dude, you good?")
                post = 0.5
            if post < 0.01: post = 0.01
            if post > 0.99: post = 0.99
            self.map.data[index] = int(post * 100)


        def update_map(dist,scan_angle,robot_angle):
            trustable_distance = 3
            #if dist == 0:
                # no return (nearest object too far or reflection failed)
                #print("thats a spicy 0!")
                
            if dist > 0:
                # reasonably certain case
                d = 0
                while d < dist - self.map.info.resolution/2:
                    trust = 0.70 - d*0.05
                    if trust < 0.51: trust = 0.51
                    update_square(d,scan_angle,robot_angle,"free",trust)
                    d = d + self.map.info.resolution
                trust = 0.9 - dist*0.1
                if trust < 0.51: trust = 0.51
                update_square(dist,scan_angle,robot_angle,"wall",trust)
            #else: print("something went terribly wrong")


        #self.map.data = [self.map.data[i] -(self.map.data[i] - 50)/50 for i in range(0,self.map.info.width * self.map.info.height)]

        scan_angle = self.lidar.angle_min
        robot_angle = get_robot_angle()
        for r in self.lidar.ranges:
            update_map(r,scan_angle,robot_angle)
            scan_angle = scan_angle + self.lidar.angle_increment
        # update_map(1,0)
        self.map.header.seq = self.map.header.seq + 1
        self.map.header.stamp = rospy.Time.now()
        self.map_publisher.publish(self.map)


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Camp_Map()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.2)
