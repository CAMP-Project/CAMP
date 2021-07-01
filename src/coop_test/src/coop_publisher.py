#!/usr/bin/env python
"""
----------------------------------------------------------------------------------------
Original Author : Sean Carda
Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
Original Date   : 2/2/2021
Description     : This python script publishes "n" amount of nodes which will count at a
                  rate x_(k+1) = x_k + n, x_0 = n. 
                  (i.e. For n = 3, x = 3, 6, 9. For n = 5, x = 5, 10, 15, 20).



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

class coop_publisher:

    # Integer n by which to start a set of numbers and to increment that list.
    global n
    n = 5

    def __init__(self):

        # Initialize node.
        rospy.init_node('coop_publisher', anonymous = False)

        # Publish "n" amount of nodes.
        self.publisherList = list()
        self.incrementList = list()

        # Generate a publisher to communicate the amount of nodes that have been created.
        self.node_count_publisher = rospy.Publisher('node_count', Float64, queue_size = 10)

        # Generate the appropriate amount of publishers.
        for i in range(1, n + 1):
            # Generate the name of each new publisher.
            name = 'coop_result_' + str(i)

            # Using the generated name, create a new publisher.
            info_publisher = rospy.Publisher(name, coop_node, queue_size = 10)

            # Add that publisher to a list.
            self.publisherList.append(info_publisher)

            # Generate a list of incrementing integers.
            self.incrementList.append(i)


    # Main functionality of the code.
    def main(self):

        # Increment the counters.
        if self.incrementList[0] >= 100:
            for i in range(0, n):
                self.incrementList[i] = (i + 1)
        else:
            for i in range(0, n):
                self.incrementList[i] += (i + 1)
        
        # Log the current iterations for debug.
        print("Print the array")
        print(str(self.incrementList))

        # Publish the amount of nodes that have been created.
        self.node_count_publisher.publish(Float64(n))

        # Publish the current counts.
        ix = 0
        for i in self.publisherList:
                result = coop_node()
                result.node_number = ix + 1
                result.value = self.incrementList[ix]
                i.publish(result)
                ix += 1


if __name__ == '__main__':
    module = coop_publisher()
    while not rospy.is_shutdown():
        module.main()
        rospy.sleep(0.1)