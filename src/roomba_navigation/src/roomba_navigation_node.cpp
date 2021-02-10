// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>
using namespace std;

//constants from teleop_key
//const int BURGER_MAX_LIN_VEL = 0.22;
//const int BURGER_MAX_ANG_VEL = 2.84;

// somewhat safer constants because i am scared
const int BURGER_MAX_LIN_VEL = 0.1;
const int BURGER_MAX_ANG_VEL = 2.00;

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
    float closest_front_object;
    int time = 0;

	// Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
	ros::Rate loop_rate(10);

	// scan_subscriber subscribes to the lidar's scan, cmd_publisher publishes a twist, and the size of the publisher queue is set to 10.
	ros::Subscriber scan_subscriber = nh.subscribe("scan", 10, msgCallback);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    	
	while(ros::ok() && time < 10*60*1) {
		// Check the cone of points for the front 55 degrees for hits, save closest hit.
        if (ranges.at(0) != 0) closest_front_object = ranges.at(0);
        else closest_front_object = 500;
        for(int i = 1; i < 27; i++) {
            if(ranges.at(i) < closest_front_object && ranges.at(i) != 0) closest_front_object = ranges.at(i);
            if(ranges.at(360-i) < closest_front_object && ranges.at(360-i) != 0) closest_front_object = ranges.at(360-i);
        }

        if (closest_front_object < 0.3) x_vel = 0;
        else if (closest_front_object < 0.5) x_vel = 0.05;
        else x_vel = 0.1;

        if (closest_front_object < 0.3) z_ang_vel = 0.5;
        else z_ang_vel = 0;

        // Write to the vel_msg we plan to publish
		vel_msg.linear.x = x_vel;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = z_ang_vel;
		
        //publish vel_msg
		cmd_publisher.publish(vel_msg);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
        time++;
	}
	return 0;
}