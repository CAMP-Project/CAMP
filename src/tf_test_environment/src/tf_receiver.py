#!/usr/bin/env python
""""
---------------------------------------------------------------------
Author: Sean Carda
Project: CAMP
Date: 7/18/2021
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


class TF_Receiver:


    def __init__(self):
        # Initialize the node.
        rospy.init_node('transformVerifier', anonymous=False)

        # Instantiate subscribers to obtain the information necessary to confirm the transform of 
        # data from odometry to decawave poses.
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/filtered', Twist, self.decaCallback)
        rospy.Subscriber('/transform', Vector3, self.transformCallback)

        # Instantiate tf2 buffer and listener.
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


        # Odometry data structures.
        self.odom_x = 0
        self.odom_y = 0
        self.odom_heading = 0

        # Decawave data structures.
        self.deca_x = 0
        self.deca_y = 0

        # Transform structure. Holds on to the entire deca <-> odom transform.
        self.frameTransform = Vector3()


    def odomCallback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        self.odom_heading = data.pose.pose.orientation.z


    def decaCallback(self, data):
        self.deca_x = data.linear.x
        self.deca_y = data.linear.y


    def transformCallback(self, data):
        self.frameTransform = data
        # *** frameTransform = [position x, position y, heading] ***


    def main(self):

        # GRAB THE TRANSFORM DATA FROM THE TF TREE AND CONVERT THE ODOMETRY POSE TO A DECAWAVE POSE                                            

        # COMPARE THE ODOM -> DECA TRANSFORM DATA TO FILTERED DECA DATA

        # COMPUTE ERROR

        # COMPARE PUBLISHED TRANSFORM TO TF BROADCASTED TRANSFORM

        # COMPUTE ERROR

        # Method to organize object pose information into a PoseStamped object for easy transformations.
        def computePose(x, y, theta, frameID):
            # Instantiate a new PoseStamped with object data.
            objectPose = PoseStamped()

            # Header information.
            objectPose.header.stamp = rospy.Time()
            objectPose.header.frame_id = frameID

            # Pose - translational information.
            objectPose.pose.position.x = x
            objectPose.pose.position.y = y
            objectPose.pose.position.z = 0

            # Pose - rotation information.
            quat = transformations.quaternion_from_euler(0, 0, theta)
            objectPose.pose.orientation.x = quat[0]
            objectPose.pose.orientation.y = quat[1]
            objectPose.pose.orientation.z = quat[2]
            objectPose.pose.orientation.w = quat[3]

            return objectPose

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


        # First, obtain the pose of the Turtlebot in its own odometry.
        odomPose = computePose(self.odom_x, self.odom_y, 0.0, 'odom')

        # Then, calculate the pose of the Turtlebot in the decawave frame. 
        odomInDecaPose = transformActions(odomPose, 'deca', 'odom', 'pose')

        # Also, grab the approximate transform used during the pose transformation.
        transformUsed = transformActions(odomPose, 'deca', 'odom', 'transform') 

        # Compare calculated Deca to actual Deca data.
        print("Calculated Deca = { x = " + str(odomInDecaPose.pose.position.x) +
                                         " y = " + str(odomInDecaPose.pose.position.y))
        print("Actual Deca     = { x = " + str(self.deca_x) +
                                         " y = " + str(self.deca_y))
        
        # Compare the retrieved transform to the published transform.
        print("TF Tree Transform   = { x = " + str(transformUsed.transform.translation.x) +
                                             " y = " + str(transformUsed.transform.translation.y) +
                                             " h = " + str(transformUsed.transform.rotation.z)) 
        print("Published Transform = { x = " + str(self.frameTransform.x) +
                                             " y = " + str(self.frameTransform.y) +
                                             " h = " + str(self.frameTransform.z)) 


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    tf_verify = TF_Receiver()
    while not rospy.is_shutdown():
        tf_verify.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)
