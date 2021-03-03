#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 2/2/2021
# Description     : This python script filters positional data from the
#                   Decawave TOF sensors. This is an extention of the standard
#                   Kalman Filter which has been specialized to handle filtering
#                   for vehicles governed by the Unicycle model. 
#
#
#
# Change Log:
#
# 2/23/2021 - Sean Carda: Created script and initial methods.
#
#
#
#----------------------------------------------------------------------------------------


# Import functions for matrix operaions.
import roslib
import csv
roslib.load_manifest('rospy')
import numpy;
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64

class EKF:

    def __init__(self):

        # Initialize node.
        rospy.init_node('kalman_filter', anonymous = False)

        self.rate = rospy.Rate(10)

        # Publish x, y, and heading.
        self.info_publisher = rospy.Publisher('filtered', Twist, queue_size = 10)

        # Subscribe to TOF Tag position, velocity, and heading.
        rospy.Subscriber('/dwm1001/tag1', Tag, self.tag_update)
        rospy.Subscriber('/odom', Odometry, self.velocity_update)
        rospy.Subscriber('/imu', Imu, self.heading_update)

        self.position = Tag()
        self.heading = float()
        self.u = 1
        self.V = float()
        self.dt = 0.1
        self.b = 1


        # State dynamics matrix. This will be a 3 x 3 matrix.
        self.A = numpy.array([[0, 0, 0], 
                              [0, 0, 0], 
                              [0, 0, 0]])

        # Input array. This will be a 3 x 1 array.
        self.B = numpy.array([[1.0], 
                              [1.0], 
                              [1.0]])

        self.C = numpy.array([[1.0, 0, 0],
                             [0, 1.0, 0],
                             [0, 0, 1.0]])

        # Process noise covariance matrix Q. This will be a 3 x 3 matrix.
        self.Q = numpy.array([[0.1, 0, 0], 
                              [0, 0.2, 0], 
                              [0, 0, 0.1]])

        # Measurement noise covariance matrix R. This will be a 3 x 3 matrix.
        self.R = numpy.array([[0.2, 0, 0], 
                              [0, 0.1, 0], 
                              [0, 0, 0.2]])

        # Uncertainty matrix. This will be a 3 x 3 matrix.
        self.P = numpy.array([[0, 0, 0], 
                              [0, 0, 0], 
                              [0, 0, 0]])

        # Finally, we will start with an initial prediction of state X = [0 0 0].
        # We will also start with a corrected state equal to the predicted state.
        self.X_hat = numpy.array([[20.0], 
                                  [20.0],
                                  [20.0]])
        self.X_corrected = self.X_hat

    # Update Tag data.
    def tag_update(self, data):
        self.position.x = data.x
        self.position.y = data.y
    
    # Update velocity data.
    def velocity_update(self, data):
        self.V = data.twist.twist.linear.x


    # Update heading data.
    def heading_update(self, data):
        self.heading = data.orientation.z

    def main(self):
        # Measurement sample:
        Y = numpy.array([[self.position.x], 
                         [self.position.y], 
                         [self.heading]])

        # Correction Step:
        K1 = numpy.dot(self.P, numpy.transpose(self.C))
        K2 = numpy.linalg.inv(numpy.dot(numpy.dot(self.C, self.P), numpy.transpose(self.C)) + self.R)
        K = numpy.dot(K1, K2)
        self.X_corrected = self.X_hat + numpy.dot(K, Y - numpy.dot(self.C, self.X_hat))
        self.P = self.P - numpy.dot(numpy.dot(K, self.C), self.P)

        # Prediction Step:

        # Linearize:
        self.A[0][2] = -1 * self.V * numpy.sin(self.heading)
        self.A[1][2] = self.V * numpy.cos(self.heading)

        dx = self.V * numpy.cos(self.heading)
        dy = self.V * numpy.sin(self.heading)
        dtheta = (self.b * self.u) * (numpy.pi / 180)
        dX = numpy.array([[dx],
                          [dy],
                          [dtheta]])
        
        self.X_hat = self.X_corrected + (dX * self.dt)
        dP = (numpy.dot(self.A, self.P) + numpy.dot(numpy.transpose(self.A), self.P) + self.Q)
        self.P = self.P + (dP * self.dt)

        # Save results and print to console. Log the same values in rqt.
        result = Twist()
        result.linear.x = self.X_corrected[0]
        result.linear.y = self.X_corrected[1]
        result.angular.x = self.X_corrected[2]

        print("\nx_i: " + str(self.position.x) + "  y_i: " + str(self.position.y) + " heading_i: " + str(self.heading) +
              "\nx_o: " + str(result.linear.x) + "  y_o: " + str(result.linear.y) + " heading_o: " + str(result.angular.x) +
              "\n")

        self.info_publisher.publish(result)

if __name__ == '__main__':
    filter = EKF()
    while not rospy.is_shutdown():
        filter.main()
        rospy.sleep(0.1)