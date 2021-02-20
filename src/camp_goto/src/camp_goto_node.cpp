/*
 * Edited lat by Tyler Pigott on 2/20/20201
 * This program should use odometry to goto a point
 * relative to where the robot started
 * whilst avoiding big obstacles using
 * the LiDAR as in in roomba_navigation.
 */


// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

// #include "localizer_dwm1001/Tag.h"

#include <vector>
#include <math.h>
#define PI 3.14159265
using namespace std;

//constants from teleop_key
//const int BURGER_MAX_LIN_VEL = 0.22;
//const int BURGER_MAX_ANG_VEL = 2.84;

// somewhat safer constants because i am scared
const float SCALE = 2;
const float BURGER_MAX_LIN_VEL = 0.2/SCALE; // 0.2 m/s
const float BURGER_MAX_ANG_VEL = 1.5708/SCALE; // pi/2 rad/s or 90 deg/s

//Movement Vars
float x_vel, z_ang_vel;
float last_x=0, last_y=0;
float long_x=0, long_y=0, cmd = 0;
float go_x, go_y, go_distance, heading;

//Scan
float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;
// float tag_x=0, tag_y=0, w, x,y,z;
float odom_x, odom_y;

// These functions run if new info was published during a spin command.
// They collects any data from the Scan, Imu, or Tag callbacks
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ROS_INFO("grabbing scan");
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    scan_time = msg->scan_time;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;
    intensities = msg->intensities;
}
// void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//     //ROS_INFO("grabbing imu");
//     heading = msg->orientation.z;
// }
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("grabbing odom");
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
}
// void tagCallback(const localizer_dwm1001::Tag::ConstPtr& msg) {
//     //ROS_INFO("grabbing tag");
//     tag_x = msg->x;
//     tag_y = msg->y;
// }

void updateLongPos(const geometry_msgs::Point::ConstPtr& msg){
    //ROS_INFO("grabbing destination");
    cmd = msg->z;
    if(cmd == 1.0){
        //Absolute mode
        long_x = msg->x;
        long_y = msg->y;
    }
    if(cmd == 2.0){
        //relative mode
        long_x = odom_x + msg->x;
        long_y = odom_y + msg->y;
    }
}

// This is the obstacle avoidance function. It is called while the robot is moving to a point to
// check for things in the way and to avoid them. 
void obstacleAvoid() {
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
    int left = 0;
    int right = 0;
    float closest_front_object;
    if(ranges.size() == 360){
        // Find how close the closest object is (55 front scans)
        if (ranges.at(0) != 0) closest_front_object = ranges.at(0);
        else closest_front_object = 500;
        for(int i = 1; i < 27; i++) {
            if(ranges.at(i) < closest_front_object && ranges.at(i) != 0) closest_front_object = ranges.at(i);
            if(ranges.at(360-i) < closest_front_object && ranges.at(360-i) != 0) closest_front_object = ranges.at(360-i);
        }
        if(closest_front_object<0.5) {
            ROS_INFO("---FOUND---");
            // Find most desirable direction, more points is better
            for(int i = 1; i < 180; i++) {
                if(ranges.at(i) < 1 && ranges.at(i) != 0) right++;
                if(ranges.at(360-i) < 1 && ranges.at(360-i) != 0) left++;
            }
            if (right<left){
                go_x = odom_x + cos(heading+PI/2);
                go_y = odom_y + sin(heading+PI/2);
            } else {
                go_x = odom_x + cos(heading-PI/2);
                go_y = odom_y + sin(heading-PI/2);
            }
        } else {
            //Don't adjust course if no object in the way
            go_x = long_x;
            go_y = long_y;
        }
    } else {
        //without LiDAR data don't move
        go_x = odom_x;
        go_y = odom_y;
    }
}

// This function is an idea I have to determine movement between two 2D points, possibly like a
// current position as determined from the dwm1001 or the kalman filter. 
void pointToPoint() {//accept a 2d point goal as a parameter and output a Twist message?
    //ROS_INFO("2p2 function");
    // Taking two points, both it's current position and either the long-term goal or
    // an intermediate step or obstacle correction point, this function determines
    // how to move smoothly between the points.

    // calculate difference between current and previous positions (dx, dy)
    float dx = odom_x - last_x;
    float dy = odom_y - last_y;
    // calculate current heading using atan2 of  dx and dy
    ROS_INFO("dy/dx: (%f/%f)",dy,dx);
    heading = atan2(dy,dx);
    // calculate the difference between the goal and the current position (gx, gy?)
    float gx = go_x - odom_x;
    float gy = go_y - odom_y;
    // find the deired heading by with atan2 of gx, gy
    float desired_heading = atan2(gy,gx);
    // find the distance remaining to travel
    go_distance = sqrt(gx*gx+gy*gy);
    // find the error in the heading
    float head_error = desired_heading - heading;
    // correct the heading error so it is between -pi and pi
    if(head_error > PI) head_error = head_error - 2*PI;
    if(head_error < 0-PI) head_error = head_error + 2*PI;
    ROS_INFO("\ndes: %f\nhed: %f\nerr: %f",desired_heading,heading,head_error);
    // control turn from heading error
    z_ang_vel = 2.0/SCALE * head_error; // 2 seems to be a good Kp in matlab sims
    // correct ang_vel so it is within acceptable margins    
    if(z_ang_vel > BURGER_MAX_ANG_VEL) z_ang_vel = BURGER_MAX_ANG_VEL;
    if(z_ang_vel < -BURGER_MAX_ANG_VEL) z_ang_vel = -BURGER_MAX_ANG_VEL;
    x_vel = BURGER_MAX_LIN_VEL; //goes fast! might be good to slow down near detected objects, but i didn't make that yet.
    if(go_distance < 0.1/SCALE) {
        //stop if at a final point
        x_vel = 0; 
        z_ang_vel = 0;
    }
    //save this for future calculations: (needs fenceposting?)
    last_x = odom_x;
    last_y = odom_y;
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
    
    //ROS_INFO("Starting the Main");

	// scan_subscriber subscribes to the lidar's scan, cmd_publisher publishes a twist, and the size of the publisher queue is set to 10.
	ros::Subscriber scan_subscriber = nh.subscribe("scan", 10, scanCallback);
    // ros::Subscriber imu_subscriber = nh.subscribe("imu", 10, imuCallback);
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, odomCallback);
    // ros::Subscriber tag_subscriber = nh.subscribe("dwm1001/tag1", 10, tagCallback);
    ros::Subscriber go_pos_subscriber = nh.subscribe("go_pos", 10, updateLongPos);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    	
	while(ros::ok()) {
        //Check inputs:
        //ROS_INFO("orientation:\nw:%f\nx:%f\ny:%f\nz:%f", w,x,y,z);
        //ROS_INFO("x: %f Y:%f", tag_x, tag_y);
        //check for obstruction and set go vars
        obstacleAvoid();
        ROS_INFO("\nAt: %f, %f\nGo: %f, %f\nDistance: %f",odom_x,odom_y,go_x,go_y,go_distance);
        //Determine motor instructions from current point and the go vars
        pointToPoint();
        ROS_INFO("X: %f  Z: %f",x_vel, z_ang_vel);
        if(cmd == 0){
            //don't move with a 0 command
            x_vel = 0; 
            z_ang_vel = 0;
        }
        // Write to the vel_msg we plan to publish
	    vel_msg.linear.x = x_vel;
	    vel_msg.linear.y = 0;
	    vel_msg.linear.z = 0;
	    vel_msg.angular.x = 0;
	    vel_msg.angular.y = 0;
	    vel_msg.angular.z = z_ang_vel;
	    //vel_msg.angular.z = 0;
        //publish vel_msg (move robot)
	    cmd_publisher.publish(vel_msg);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
	return 0;
}