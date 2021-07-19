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
roslib.load_manifest('rospy')
#------------------------------------------------------------------------------------
# Standard msg Imports.
from geometry_msgs.msg import PoseStamped, Twist, Vector3
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
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)


        # Odometry data structures.
        self.odom_x = 0
        self.odom_y = 0
        self.odom_heading = 0

        # Decawave data structures.
        self.deca_x = 0
        self.deca_y = 0

        # Transform structure. Holds on to the entire deca <-> odom transform.
        self.frameTransform


    def odomCallback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        self.odom_heading = data.pose.pose.orientation.z


    def decaCallback(self, data):
        self.deca_x = data.x
        self.deca_y = data.y


    def transformCallback(self, data):
        self.frameTransform = data
        # *** frameTransform = [position x, position y, heading] ***


    def main():

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


        def computeTransform(objectPoseOne, objectPoseTwo)



        print("This is temporary")








# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    tf_verify = TF_Receiver()
    while not rospy.is_shutdown():
        tf_verify.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)
