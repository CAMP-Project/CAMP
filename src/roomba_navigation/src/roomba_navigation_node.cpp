// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>
using namespace std;

// Declare Scan variables globaly
float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;

// This function runs if new info was published during a spin command.
// It collects any data from the Scan message
void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    scan_time = msg->scan_time;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;
    intensities = msg->intensities;
}

// This is the main function.
// It contains some code to run once and a while loop for repeatign actions.
int main(int argc, char **argv){
    // Name the node
	ros::init(argc, argv, "roomba_brian");
	ros::NodeHandle nh;	

    // This is the message to send that contains motor instructions.
	geometry_msgs::Twist vel_msg; 

    // Declare variables for vel_cmd message
    float x_vel = 0, z_ang_vel = 0;
    bool safe;

	// Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
	ros::Rate loop_rate(10);

	// scan_subscriber subscribes to the lidar's scan, cmd_publisher publishes a twist, and the size of the publisher queue is set to 10.
	ros::Subscriber scan_subscriber = nh.subscribe("scan", 10, msgCallback);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    	
	while(ros::ok()) {
		// Check the cone of points for the front 55 degrees for hits within 0.3m
        safe = true;
        for(int i = 360-27; i < 360+27; i++) 
            if(ranges.at(i%360) < 0.3) safe = false;

        if(safe) x_vel = 0.1;

        // Write to the vel_msg we plan to publish
		vel_msg.linear.x = vel;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = spin;
		
        //publish vel_msg
		cmd_publisher.publish(vel_msg);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
	return 0;