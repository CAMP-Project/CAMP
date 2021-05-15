/*
 * Edited lat by Tyler Pigott on 2/20/20201
 * This program tells the camp_goto_node where
 * the robot's long-term goal is.
 */

// ROS Default Header File
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "camp_goto/Cmd.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv){
    // Name the node
	ros::init(argc, argv, "camp_talker");
	ros::NodeHandle nh;	

	camp_goto::Cmd go_cmd; 

    // Declare variables for vel_cmd message
    float x=0, y=0, speed=0;

	ros::Rate loop_rate(10);
	ros::Publisher cmd_publisher = nh.advertise<camp_goto::Cmd>("go_cmd", 10);
    	
	while(ros::ok()) {
        cout << "Please enter an x coord, y coord, and speed.\n";
        cout << "Decawave is lame, all these poitns are in absolute odometry.\n";
        cout << "Recomended speed is 0.45. Speed of 0 stops the robot (obviously).\n";
        cin >> x >> y >> speed;
		
	    go_cmd.destination.x = x;
	    go_cmd.destination.y = y;
	    go_cmd.destination.z = 0;

		go_cmd.speed = speed;

		go_cmd.stop = false;
		go_cmd.is_deca = false;
		go_cmd.is_relative = false;

		go_cmd.destination_stop_distance = 0.08 * speed + 0.02;
		go_cmd.emergency_stop_angle = 30;
		go_cmd.emergency_stop_distance = 0.2;

	    cmd_publisher.publish(go_cmd);
        loop_rate.sleep();
	}
	return 0;
}