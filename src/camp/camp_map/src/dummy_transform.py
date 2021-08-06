#!/usr/bin/env python
""""
---------------------------------------------------------------------
Author: Sean Carda
Project: CAMP
Date: 7/20/2021
---------------------------------------------------------------------
"""

#------------------------------------------------------------------------------------
# Standard ROS Imports.
import roslib
import math
import rospy
import tf2_py
roslib.load_manifest('rospy')
#------------------------------------------------------------------------------------
# Standard msg Imports.
from geometry_msgs.msg import PoseStamped, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
#------------------------------------------------------------------------------------
# TF2 Imports.
import tf2_geometry_msgs
import tf2_msgs
import tf2_ros
from tf import transformations
#------------------------------------------------------------------------------------


class Calibrator_Python:

    def __init__(self):
        
        # Initialize the node.
        rospy.init_node('Python_Calibration', anonymous=False)

        # Setup a tf buffer and broadcaster.
        self.tfBuffer = tf2_ros.Buffer()
        self.broadcaster = tf2_ros.TransformBroadcaster()


    # Functionality.
    def main(self):
        # Prepare a transform stamped for tf publishing.
        transformToBroadcast = TransformStamped()
        
        # Transform header and child ID info.
        transformToBroadcast.header.stamp = rospy.Time.now()
        transformToBroadcast.header.frame_id = "odom"
        transformToBroadcast.child_frame_id = "testframe"

        # Transform translational info.
        transformToBroadcast.transform.translation.x = -2
        transformToBroadcast.transform.translation.y = 5
        transformToBroadcast.transform.translation.z = 0.0

        # Transform rotational info.
        theta = 0.785
        quat = transformations.quaternion_from_euler(0, 0, theta)
        transformToBroadcast.transform.rotation.x = quat[0]
        transformToBroadcast.transform.rotation.y = quat[1]
        transformToBroadcast.transform.rotation.z = quat[2]
        transformToBroadcast.transform.rotation.w = quat[3]

        # Return the calculated transform.
        self.broadcaster.sendTransform(transformToBroadcast)
        

if __name__== '__main__':
    calibrator = Calibrator_Python()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        calibrator.main()