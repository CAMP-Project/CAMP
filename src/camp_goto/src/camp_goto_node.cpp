/*
 * Edited lat by Tyler Pigott on 2/18/20201
 * This program should use the decawaves to goto a point
 * on the decawave grid, whilst avoiding big obstacles using
 * the LiDAR like in roomba_navigation.
 */


// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include "localizer_dwm1001/Tag.h"

#include <vector>
#include <math.h>
#define PI 3.14159265
using namespace std;

//constants from teleop_key
//const int BURGER_MAX_LIN_VEL = 0.22;
//const int BURGER_MAX_ANG_VEL = 2.84;

// somewhat safer constants because i am scared
const int BURGER_MAX_LIN_VEL = 0.2; // 2 m/s
const int BURGER_MAX_ANG_VEL = 1.5708; // pi/2 rad/s or 90 deg/s

// Declare Scan variables globaly
float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
float tag_x=0, tag_y=0, heading;
float last_x=0, last_y=0;
vector<float> ranges, intensities;

// These functions run if new info was published during a spin command.
// They collects any data from the Scan, Imu, or Tag callbacks
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    scan_time = msg->scan_time;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;
    intensities = msg->intensities;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    heading = msg->orientation.z;
}
void tagCallback(const localizer_dwm1001::Tag::ConstPtr& msg) {
    tag_x = msg->x;
    tag_y = msg->y;
}

// This is the obstacle avoidance function. It is called while the robot is moving to a point to
// check for things in the way and to avoid them. 
// void obstacleAvoid() { // Change to return a 2d point (vector?)?
    // ROS topic variables are already global, should be available for this function
    // I am going to write some pseudocode to help plan this function
    //--PSEUDOCODE--
    // Knowing the path to be taken, check for objects in the way
    // if(object found) {
        // center scan on closest direction toward the final point
        // scan for closest opening around object (r/l decider from roomba, centered on desired heading)
        // return path around object
    // }
    // else return original path
// }

// This function is an idea I have to determine movement between two 2D points, possibly like a
// current position as determined from the dwm1001 or the kalman filter. 
void pointToPoint(float go_x, float go_y) {//accept a 2d point goal as a parameter and output a Twist message?
    // Taking two points, both it's current position and either the long-term goal or
    // an intermediate step or obstacle correction point, this function determines
    // how to move smoothly between the points.
    geometry_msgs::Twist vel_msg;

    // calculate difference between current and previous positions (dx, dy)
    float dx = tag_x - last_x;
    float dy = tag_y - last_y;
    // calculate current heading using atan2 of  dx and dy
    float current_heading = atan2(dy,dx);
    // calculate the difference between the goal and the current position (gx, gy?)
    float gx = go_x - tag_x;
    float gy = go_x - tag_y;
    // find the deired heading by with atan2 of gx, gy
    float desired_heading = atan2(gy,gx);
    // find the distance remaining to travel
    float go_distance = sqrt(gx*gx+gy*gy);
    // find the error in the heading
    float head_error = current_heading - desired_heading;
    // correct the heading error so it is between -pi and pi
    if(head_error > PI) head_error = head_error - 2*PI;
    if(head_error < 0-PI) head_error = head_error + 2*PI;
    // control turn from heading error
    z_ang_vel = 2 * head_error; // 2 seems to be a good Kp in matlab sims
    // correct ang_vel so it is within acceptable margins    
    if(z_ang_vel > BURGER_MAX_ANG_VEL) z_ang_vel = BURGER_MAX_ANG_VEL;
    x_vel = BURGER_MAX_LIN_VEL; //goes fast! might be good to slow down near detected objects, but i didn't make that yet.
    if(go_distance < 0.1) x_vel = 0; //stop if at a final point
    // Write to the vel_msg we plan to publish
	vel_msg.linear.x = x_vel;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = z_ang_vel;
    //publish vel_msg (move robot)
	cmd_publisher.publish(vel_msg);
    //save this for future calculations: (needs fenceposting?)
    last_x = tag_x;
    last_y = tag_y;
}

// This is the main function.
// It contains some code to run once and a while loop for repeating actions.
int main(int argc, char **argv){
    // Name the node
	ros::init(argc, argv, "brian");
	ros::NodeHandle nh;	

    // This is the message to send that contains motor instructions.
	geometry_msgs::Twist vel_msg; 

	// Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
	ros::Rate loop_rate(10);

	// scan_subscriber subscribes to the lidar's scan, cmd_publisher publishes a twist, and the size of the publisher queue is set to 10.
	ros::Subscriber scan_subscriber = nh.subscribe("scan", 10, scanCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("imu", 10, imuCallback);
    ros::Subscriber tag_subscriber = nh.subscribe("dwm1001/tag1", 10, tagCallback);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    	
	while(ros::ok()) {
        //Check inputs:
        ROS_INFO("heading: %f", heading);
        ROS_INFO("x: %f Y:%f", tagx, tagy);



       
        //pointToPoint(1,1);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
	return 0;
}