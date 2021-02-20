/*
 * Edited lat by Tyler Pigott on 2/20/20201
 * This program tells the camp_goto_node where
 * the robot's long-term goal is.
 */

// ROS Default Header File
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv){
    // Name the node
	ros::init(argc, argv, "brian_ears");
	ros::NodeHandle nh;	

	geometry_msgs::Point go_pos; 

    // Declare variables for vel_cmd message
    float x=0, y=0, cmd=0;

	ros::Rate loop_rate(10);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Point>("go_pos", 10);
    	
	while(ros::ok()) {
        cout << "Please enter an x coord, y coord, and execution mode.\n";
        cout << "execution mode 0 is no movement, 1 is absolute movement, and 2 is relative movement.\n";
        cin >> x >> y >> cmd;
	    go_pos.x = x;
	    go_pos.y = y;
	    go_pos.z = cmd;
	    cmd_publisher.publish(go_pos);
        loop_rate.sleep();
	}
	return 0;
}