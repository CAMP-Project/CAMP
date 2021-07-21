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
        rospy.Publisher('/transform', Vector3, queue_size=10)

        # Setup constants for odometry data and decawave data.
        self.odom_x = 0
        self.odom.y = 0
        self.deca_x = 0
        self.deca_y = 0

        self.odom_x_list = []
        self.odom_y_list = []
        self.deca_x_list = []
        self.deca_y_list = []

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
        while len(self.odom_x_list) > (self.max_points_modifier - self.max_points_modification):
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
                for i in range(0, n):
                    coeff1 = (ty - self.deca_y_list[i]) * (a * self.odom_x_list[i] - b * self.odom_y_list[i])
                    coeff2 = (tx - self.deca_x_list[i]) * (a * self.odom_y_list[i] + b * self.odom_x_list[i])
                    f = f + coeff1 - coeff2

                    coeff1a = (ty2 - self.deca_y_list[i]) * (a2 * self.odom_x_list[i] - b2 * self.odom_y_list[i])
                    coeff2a = (tx2 - self.deca_x_list[i]) * (a2 * self.odom_y_list[i] + b2 * self.odom_x_list[i])
                    f2 = f2 + coeff1a - coeff2a

                f_prime = (f2 - f) / self.d_theta
                last_theta = theta
                theta = theta - (f / f_prime)








if __name__== '__main__':
    calibrator = Calibrator_Python()
    while not rospy.is_shutdown():
        calibrator.main()

        # Run at 10 Hz
        rospy.sleep(0.1)