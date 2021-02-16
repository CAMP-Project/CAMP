#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 2/2/2021
# Description     : This python script filters positional data from the
#                   Decawave TOF sensors.  
#
#
#
# Change Log:
#
# 2/2/2021 - Sean Carda: Created script and initial methods.
#
#
#
#----------------------------------------------------------------------------------------


# Import functions for matrix operaions.
import roslib
roslib.load_manifest('rospy')
import numpy;
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64

class KalmanFilter:

    def __init__(self):
        # Initialize node.
        rospy.init_node('kalman_filter', anonymous = False)

        # Publish x, y, and heading.
        self.info_publisher = rospy.Publisher('filtered', Twist, queue_size = 10)

        # Subscribe to TOF Tag position, velocity, and heading.
        rospy.Subscriber('/dwm1001/tag1', Tag, self.tag_update)
        rospy.Subscriber('/odom', Twist, self.velocity_update)
        rospy.Subscriber('/imu', Float64, self.heading_update)

        self.position = Tag()
        self.heading = float()
        self.u = float()

        # Set update rate to 10 Hz.
        self.rate = rospy.Rate(1)

        # State dynamics matrix. This will be a 3 x 3 matrix.
        self.A = numpy.array([[1, 1, 1], 
                              [1, 1, 1], 
                              [1, 1, 1]])

        # Input array. This will be a 3 x 1 array.
        self.B = numpy.array([1.0, 
                              1.0, 
                              1.0])

        # Input measurement matrix. This will be a 3 x 3 matrix.
        self.C = numpy.array([[1, 1, 1], 
                              [1, 1, 1], 
                              [1, 1, 1]])

        # Process noise covariance matrix Q. This will be a 3 x 3 matrix.
        self.Q = numpy.array([[0.1, 0, 0], 
                              [0, 0.2, 0], 
                              [0, 0, 0.3]])

        # Measurement noise covariance matrix R. This will be a 3 x 3 matrix.
        self.R = numpy.array([[0.2, 0, 0], 
                              [0, 0.5, 0], 
                              [0, 0, 0.7]])

        # Uncertainty matrix. This will be a 3 x 3 matrix.
        self.P = numpy.array([[0, 0, 0], 
                              [0, 0, 0], 
                              [0, 0, 0]])

        # Finally, we will start with an initial prediction of state X = [0 0 0].
        # We will also start with a corrected state equal to the predicted state.
        self.X_hat = numpy.array([0, 
                                  0,
                                  0])
        self.X_corrected = self.X_hat

    # Update Tag data.
    def tag_update(self, data):
        self.position.x = data.x
        self.position.y = data.y
    
    # Update velocity data.
    def velocity_update(self, data):
        self.u = data.twist.linear.x.data

    # Update heading data.
    def heading_update(self, data):
        self.heading = data.orientation.z.data


    def main(self):

        # Predict the state:
        self.X_hat = numpy.dot(self.A, self.X_corrected) + (self.u * self.B)

        # Calculate new covariance:
        self.P = numpy.dot(self.A, numpy.dot(self.A, self.P)) + self.Q

        # Calculate Kalman Gain:
        K1 = numpy.linalg.inv(numpy.dot(self.C, numpy.dot(self.P, self.C)) + self.R)
        K = numpy.dot(self.P, numpy.dot(self.C, K1))
        
        # Measurement Y:
        Y = numpy.array([self.position.x, 
                         self.position.y, 
                         self.heading])

        # Correct the prediction:
        self.X_corrected = self.X_hat + (numpy.dot(K, Y) - numpy.dot(self.C, self.X_hat))

        # Correct covariance:
        self.P = self.P - numpy.dot(K, numpy.dot(self.C, self.P))

        result = Twist()
        result.linear.x = self.X_corrected[0]
        result.linear.y = self.X_corrected[1]
        result.angular.x = self.X_corrected[2]

        self.info_publisher.publish(result)

if __name__ == '__main__':
    filter = KalmanFilter()
    while not rospy.is_shutdown():
    
        try:    
            filter.main()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    


    









