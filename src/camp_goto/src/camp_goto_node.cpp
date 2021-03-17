/*
 * Edited last by Tyler Pigott on 3/2/2021
 * This program should use odometry to goto a point
 * relative to where the robot started
 * whilst avoiding big obstacles using
 * the LiDAR as in in roomba_navigation.
 * 
 * Reverted back to an older version for stability, development ended 3/17/20201.
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
float long_x=0, long_y=0; 
int cmd = 0;
float go_x, go_y, go_distance, heading;
//Scan
float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;
//Odometry and Decawave
float odom_x, odom_y;
float deca_x, deca_y;
float offset_x=0, offset_y=0, offset_theta=0;
float lastdeca_x, lastdeca_y, lastodom_x, lastodom_y;
float decagoal_x, decagoal_y;

float decaToOdomX(float x, float y){
    return (x-offset_x)*cos(offset_theta)+(y-offset_y)*sin(offset_theta);
}
float decaToOdomY(float x, float y){
    return (y-offset_y)*cos(offset_theta)-(x-offset_x)*sin(offset_theta);
}

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
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("grabbing odom");
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
}
void tagCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("grabbing tag");
    deca_x = msg->linear.x;
    deca_y = msg->linear.y;
}
/**
 * This function determines the point the robot
 * will travel to next. It runs each time the 
 * talker sends a point.
 */
void updateLongPos(const geometry_msgs::Point::ConstPtr& msg){
    //ROS_INFO("grabbing destination");
    cmd = msg->z;
    if(cmd == 1 || cmd ==2){
        long_x = msg->x;
        long_y = msg->y;
    }
    if(cmd == 2){
        //relative mode
        long_x += odom_x;
        long_y += odom_y;
    }
    if(cmd == 3){
        //convert decawave to odometry
        decagoal_x = msg->x;
        decagoal_y = msg->y;
        long_x = decaToOdomX(decagoal_x,decagoal_y);
        long_y = decaToOdomY(decagoal_x,decagoal_y);
    }
    if(cmd ==4){
        //convert decawave to odometry (relative to current decawave location)
        decagoal_x = deca_x+msg->x;
        decagoal_y = deca_y+msg->y;
        long_x = decaToOdomX(decagoal_x,decagoal_y);
        long_y = decaToOdomY(decagoal_x,decagoal_y);
    }
}

// This is the obstacle avoidance function. It is called while the robot is moving to a point to
// check for things in the way and to avoid them. 
// TODO: update to fix corner-cutting problem
void obstacleAvoid() {
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
/**
 * This function navigates the robot to the point (go_x,go_y) based on the odometry.
 */
void pointToPoint() {
    float dx = odom_x - last_x;
    float dy = odom_y - last_y;
    ROS_INFO("dy/dx: (%f/%f)",dy,dx);
    if(!(dy == 0 && dx == 0)) heading = atan2(dy,dx);
    float gx = go_x - odom_x;
    float gy = go_y - odom_y;
    float desired_heading = atan2(gy,gx);
    go_distance = sqrt(gx*gx+gy*gy);
    
    float head_error = desired_heading - heading;
    if(head_error > PI) head_error = head_error - 2*PI;
    if(head_error < 0-PI) head_error = head_error + 2*PI;
    ROS_INFO("\ndes: %f\nhed: %f\nerr: %f",desired_heading,heading,head_error);
    
    //TODO: implement better control
    z_ang_vel = 2.0/SCALE * head_error; // 2 seems to be a good Kp in matlab sims
    if(z_ang_vel > BURGER_MAX_ANG_VEL) z_ang_vel = BURGER_MAX_ANG_VEL;
    if(z_ang_vel < -BURGER_MAX_ANG_VEL) z_ang_vel = -BURGER_MAX_ANG_VEL;
    x_vel = BURGER_MAX_LIN_VEL*(0.5+0.5*(1-abs(head_error)/PI));
    if(go_distance < 0.1/SCALE) {
        //stop if at a final point
        x_vel = 0; 
        z_ang_vel = 0;
    }
    last_x = odom_x;
    last_y = odom_y;
}

void checkOrientation() {
    //calculate the decawave approximation
    float calc_x = odom_x*cos(offset_theta)-odom_y*sin(offset_theta)+offset_x;
    float calc_y = odom_y*cos(offset_theta)+odom_x*sin(offset_theta)+offset_y;
    // if approximation is bad, recalc. 
    ROS_INFO("\nGoalDeca: (%f,%f)\nCalcDeca: (%f,%f)\nFiltDeca: (%f, %f)",decagoal_x,decagoal_y,calc_x,calc_y,deca_x,deca_y);
    if(sqrt(pow(calc_x-deca_x,2)+pow(calc_y-deca_y,2))>1){
        ROS_INFO("recalculating... %f",sqrt(pow(calc_x-deca_x,2)+pow(calc_y-deca_y,2)));
        offset_x=deca_x-odom_x;
        offset_y=deca_y-odom_y;
        offset_theta = atan2(deca_y-lastdeca_y,deca_x-lastdeca_x)-atan2(odom_y-lastodom_y,odom_x-lastodom_x);
        lastdeca_x = deca_x;
        lastdeca_y = deca_y;
        lastodom_x = odom_x;
        lastodom_y = odom_y;
        //update goal with new deca conversion if in decawave navigation mode
        if (cmd >= 3){
            long_x = decaToOdomX(decagoal_x,decagoal_y);
            long_y = decaToOdomY(decagoal_x,decagoal_y);
        }
    }
    ROS_INFO("\noffset: (%f,%f) @ %f rad",offset_x,offset_y,offset_theta);
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
    ros::Subscriber tag_subscriber = nh.subscribe("filtered", 10, tagCallback);
    ros::Subscriber go_pos_subscriber = nh.subscribe("go_pos", 10, updateLongPos);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    	
	while(ros::ok()) {
        //make sure decawave-odometry translation is accurate
        checkOrientation();
        //check for obstruction and set go vars
        obstacleAvoid();
        ROS_INFO("\ncmd:%i\nAt: %f, %f\nGo: %f, %f\nDistance: %f",cmd,odom_x,odom_y,go_x,go_y,go_distance);
        
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