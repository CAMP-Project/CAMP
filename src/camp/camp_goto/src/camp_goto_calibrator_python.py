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

        # Setup subscribers for odometry and decawave data.
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/filtered', Twist, self.decaCallback)

        # Setup publisher for sending transform data.
        self.transform_publisher = rospy.Publisher('/transform', Vector3, queue_size=10)

        # Setup a tf buffer and broadcaster.
        self.tfBuffer = tf2_ros.Buffer()
        self.broadcaster = tf2_ros.TransformBroadcaster()


        # Setup constants for odometry data and decawave data.
        self.odom_x = 0
        self.odom_y = 0
        self.deca_x = 0
        self.deca_y = 0

        self.odom_x_list = [0]
        self.odom_y_list = [0]
        self.deca_x_list = [0]
        self.deca_y_list = [0]

        # Parameter information for the Least Squares Best Fit algorithm.
        self.min_points = 3                  # The minimum number of points to allow in the lists.
        self.max_points = 99                 # The maximum number of points to allow in the lists.
        self.max_points_modifier = 0         # Amount of points in the front of the point lists to remove.
        self.max_points_modification = 15    #
        self.new_point_distance = 0.1
        self.bad_theta_threshold = 0.2
        self.bad_data_threshold = 5
        self.theta_found_threshold = 0.001
        self.d_theta = 0.0001
        self.theta_0 = 0

        # Keep a reference of the actual transform which will be broadcast by tf.
        self.transform = TransformStamped()

        self.transform.header.stamp = rospy.Time()
        self.transform.header.frame_id = "odom"
        self.transform.child_frame_id = "deca"
        
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.0

        # Transform rotational info.
        quat = transformations.quaternion_from_euler(0, 0, 0)
        self.transform.transform.rotation.x = quat[0]
        self.transform.transform.rotation.y = quat[1]
        self.transform.transform.rotation.z = quat[2]
        self.transform.transform.rotation.w = quat[3]


    # Odometry callback.
    def odomCallback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        

    # Decawave callback. Grabs filtered data.
    def decaCallback(self, data):
        self.deca_x = data.linear.x
        self.deca_y = data.linear.y


    # If there are points that need to be removed or added, remove or add them.
    def shiftCoords(self):
        self.odom_x_list.append(self.odom_x)
        self.odom_y_list.append(self.odom_y)
        self.deca_x_list.append(self.deca_x)
        self.deca_y_list.append(self.deca_y)
        while len(self.odom_x_list) > (self.max_points - self.max_points_modifier):
            self.odom_x_list.pop()
            self.odom_y_list.pop()
            self.deca_x_list.pop()
            self.deca_y_list.pop()


    # Functionality.
    def main(self):

        # Method for computing the transform from odometry to decawave frames.
        def getOffsets(lastTransform):
            # Initialize theta and 'last' theta.
            theta = self.theta_0
            last_theta = self.theta_0
            
            # Indicator to stay in the while loop.
            run = 1

            attempt = 1

            while run is 1:
                run = 0
                a = math.cos(theta)
                b = math.sin(theta)

                a2 = math.cos(theta + self.d_theta)
                b2 = math.sin(theta + self.d_theta)

                # Old translation of the transform in x and y.
                tx = 0
                ty = 0

                # New translation of the transform in x and y.
                tx2 = 0
                ty2 = 0

                # Size of the current list of points.
                n = len(self.odom_x_list)

                # First summation to compute the translational transformation offsets.
                for i in range(0, n):
                    tx = tx + (self.deca_x_list[i] - (a * self.odom_x_list[i]) + (b * self.odom_y_list[i]))
                    ty = ty + (self.deca_y_list[i] - (a * self.odom_y_list[i] - (b * self.odom_x_list[i])))
                    tx2 = tx2 + (self.deca_x_list[i] - (a2 * self.odom_x_list[i]) + (b2 * self.odom_y_list[i]))
                    ty2 = ty2 + (self.deca_y_list[i] - (a2 * self.odom_y_list[i]) - (b2 * self.odom_x_list[i]))
                
                # Normalize the translational offsets.
                tx = tx / n
                ty = ty / n
                tx2 = tx2 / n
                ty2 = ty2 / n

                f = 0
                f2 = 0

                # Second summation to compute the orientational transformation offset.
                for i in range(0, n):
                    coeff1 = (ty - self.deca_y_list[i]) * (a * self.odom_x_list[i] - b * self.odom_y_list[i])
                    coeff2 = (tx - self.deca_x_list[i]) * (a * self.odom_y_list[i] + b * self.odom_x_list[i])
                    f = f + coeff1 - coeff2

                    coeff1a = (ty2 - self.deca_y_list[i]) * (a2 * self.odom_x_list[i] - b2 * self.odom_y_list[i])
                    coeff2a = (tx2 - self.deca_x_list[i]) * (a2 * self.odom_y_list[i] + b2 * self.odom_x_list[i])
                    f2 = f2 + coeff1a - coeff2a

                #TODO:
                # 1) Comment the purposes of these constants.

                f_prime = (f2 - f) / self.d_theta
                last_theta = theta
                theta = theta - (f / f_prime)

                if f_prime is 0:
                    f_prime = 0.00000001

                if theta > math.pi or theta < -math.pi:
                    theta = theta - (math.floor(theta / math.pi) * math.pi)

                if theta is 0:
                    theta = 0.00000001

                if abs((theta - last_theta)/theta) > self.theta_found_threshold:
                    run = 1
                else:

                    # Third loop to compute the error function.
                    e = 0
                    for i in range(0, n):
                        firstPower = math.pow((a * self.odom_x_list[i]) - (b * self.odom_y_list[i]) + (tx - self.deca_x_list[i]), 2)
                        secondPower = math.pow((b * self.odom_x_list[i]) + (a * self.odom_y_list[i]) + (ty - self.deca_y_list[i]), 2)
                        e = e + firstPower + secondPower
                    
                    # Normalize the error function.
                    e = e / n
                    
                    # Some debug information.
                    rospy.loginfo("-- Attempt:" + str(attempt))
                    rospy.loginfo("Error     :" + str(e))
                    rospy.loginfo("Theta     :" + str(theta))

                    if e > self.bad_theta_threshold:
                        if theta is not 0:
                            theta = theta - (theta / abs(theta)) * math.pi
                        else:
                            theta = math.pi
                        run = 1
                        attempt = attempt + 1
                        if attempt > 3:
                            self.max_points_modifier = self.max_points_modification
                            return lastTransform


            self.theta_0 = theta

            # Prepare a transform stamped for tf and a vector3 for publishing.
            transformToBroadcast = TransformStamped()
            transformToPublish = Vector3(tx, ty, theta)
            
            # Transform header and child ID info.
            transformToBroadcast.header.stamp = rospy.Time()
            transformToBroadcast.header.frame_id = "odom"
            transformToBroadcast.child_frame_id = "deca"

            # Transform translational info.
            transformToBroadcast.transform.translation.x = tx
            transformToBroadcast.transform.translation.y = ty
            transformToBroadcast.transform.translation.z = 0.0

            # Transform rotational info.
            quat = transformations.quaternion_from_euler(0, 0, theta)
            transformToBroadcast.transform.rotation.x = quat[0]
            transformToBroadcast.transform.rotation.y = quat[1]
            transformToBroadcast.transform.rotation.z = quat[2]
            transformToBroadcast.transform.rotation.w = quat[3]

            # Publish and broadcast the transformStamped and vector3.
            self.transform_publisher.publish(transformToPublish)

            # Return the calculated transform.
            return transformToBroadcast


        # Perform this action if there are no points yet.
        if len(self.odom_x_list) is 0:
            rospy.loginfo("First Push!")
            self.shiftCoords()
            rospy.loginfo("Size: " + len(self.odom_x_list))

        # Perform this action if it is time to aquire another point.
        # First, aquire the difference between the last point stored and the most recent point to come in.
        diff = math.sqrt(pow(self.odom_x_list[-1] - self.odom_x, 2) + pow(self.odom_y_list[-1] - self.odom_y, 2))

        # If that distance is sufficiently above the minimum new points distance, and if the list is not empty, then proceed.
        if diff > self.new_point_distance and len(self.odom_x_list) is not 0:
            self.shiftCoords()
            rospy.loginfo("Size: " + str(len(self.odom_x_list)))

            # If there are sufficiently many points in the list to perform a calculation.
            if len(self.odom_x_list) > self.min_points:
                rospy.loginfo("Calculating new offsets...")
                self.transform = getOffsets(self.transform)

        self.broadcaster.sendTransform(self.transform)
        




if __name__== '__main__':
    calibrator = Calibrator_Python()
    while not rospy.is_shutdown():
        calibrator.main()

        # Run at 10 Hz
        rospy.sleep(0.1)