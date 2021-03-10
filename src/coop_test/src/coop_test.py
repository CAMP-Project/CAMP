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
roslib.load_manifest('rospy')
import numpy;
import rospy
from std_msgs.msg import String, Float64

class COOP:

    def __init__(self):

        # Initialize node.
        rospy.init_node('coop_test', anonymous = False)

        # Initialize the ROS rate.
        self.rate = rospy.Rate(1)

        # Publish x, y, and heading.
        self.info_publisher = rospy.Publisher('test_result', Float64, queue_size = 10)
        
        # Initialize test result. This will be accessed by another multimaster core.
        self.result = Float64()
        self.result.data = 1


    def main(self):
        # Simple program. Icrement until a certain point, then reset.
        if self.result.data == 100:
            self.result.data = 1
        else:
            self.result.data += 1

        # Print debug and publish.
        print("result = " + str(self.result.data))
        self.info_publisher.publish(self.result)

if __name__ == '__main__':
    test = COOP()
    while not rospy.is_shutdown():
        test.main()
        rospy.sleep(0.1)