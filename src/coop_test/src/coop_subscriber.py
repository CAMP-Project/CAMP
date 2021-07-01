#!/usr/bin/env python
"""
----------------------------------------------------------------------------------------
Original Author : Sean Carda
Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
Original Date   : 2/2/2021
Description     : This python script subscribes to the nodes created by coop_publisher.



Change Log:

6/30/2021 - Sean Carda: Created script and initial methods.



----------------------------------------------------------------------------------------
"""


# Import functions for matrix operaions.
import roslib
roslib.load_manifest('rospy')
import rospy
from std_msgs.msg import Float64

class coop_publisher:


    def __init__(self):

        # Initialize node.
        rospy.init_node('coop_subscriber', anonymous = False)

        # Create a subscriber for the node count created by the coop_publisher script.
        
    def updateSubscriber(self, data):
        print("This is temporary")




    # Main functionality of the code.
    def main(self):
        print("This is temporary")

        


if __name__ == '__main__':
    module = coop_publisher()
    while not rospy.is_shutdown():
        module.main()
        rospy.sleep(0.1)