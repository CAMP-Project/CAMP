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

        # Setup subscribers for odometry and decawave data.
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/filtered', Twist, self.decaCallback)



    def odomCallback(self, data):
        print('This is temporary')


    def decaCallback(self, data):
        print('This is temporary')


    def main(self):
        print('Tis temporary')




if __name__== '__main__':
    calibrator = Calibrator_Python()
    while not rospy.is_shutdown():
        calibrator.main()

        # Run at 10 Hz
        rospy.sleep(0.1)