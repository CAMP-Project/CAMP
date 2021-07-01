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


# Imports.
import roslib
roslib.load_manifest('rospy')
import rospy
from std_msgs.msg import Float64
from coop_test.msg import coop_node

class coop_subscriber:


    def __init__(self):

        # Initialize node.
        rospy.init_node('coop_subscriber', anonymous = False)

        # Storage and subscriber for node count. Wait for 1 second while the data is being fetched to
        # prevent errors in generating data subscribers.
        self.nodeCount = 0
        rospy.Subscriber('/node_count', Float64, self.updateNodeCount)
        rospy.sleep(1)


        # Create subscribers for every node produced by the coop publisher.
        self.grabbedValues = [None] * self.nodeCount
        for i in range(1, self.nodeCount + 1):
            name = '/coop_result_' + str(i)
            rospy.Subscriber(name, coop_node, self.updateSubscriber)


    # Callback for updating the data produced by the coop publisher.
    def updateSubscriber(self, data):
        self.grabbedValues[int(data.node_number) - 1] = int(data.value)


    # Callback for updating the node count output by he coop publisher.
    def updateNodeCount(self, data):
        self.nodeCount = int(data.data)


    # Main functionality of the code.
    def main(self):
        print("Print the array")
        print(str(self.grabbedValues))

        
if __name__ == '__main__':
    module = coop_subscriber()
    while not rospy.is_shutdown():
        module.main()
        rospy.sleep(0.1)