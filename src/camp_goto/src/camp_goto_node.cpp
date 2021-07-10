/*
 * Edited last by Tyler Pigott on 4/22/2021
 * Some changes were made to make the robot faster,
 * give more useful debug info,
 * and recalculate the goal when navigating in modes 3/4 and transform updates.
 */

// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "camp_goto/Cmd.h"
#include <vector>
#include <math.h>
#include <std_msgs/Bool.h>
#define PI 3.14159265

#include <signal.h>

using namespace std;
//constants from teleop_key
const float BURGER_MAX_LIN_VEL = 0.22;
const float BURGER_MAX_ANG_VEL = 2.84;
// somewhat safer constants because i am scared
//const float BURGER_MAX_LIN_VEL = 0.2; // 0.2 m/s
//const float BURGER_MAX_ANG_VEL = 1.5708; // pi/2 rad/s or 90 deg/s

struct command{
    bool stop = true;
    bool is_relative = false;
    bool is_deca = false;
} cmd;

struct parameters{
    float speed = 0.45;
    float dest_stop = 0.1;
    float emg_stop = 0.15;
    int emeg_stop_angle = 27;
} param;

geometry_msgs::Twist vel_msg; 

//Movement Vars
float x_vel, z_ang_vel;
float last_x=0, last_y=0;
float go_x, go_y, go_distance, heading;
float velocity_direction = 1;
//Scan
float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;
//Odometry and Decawave
float odom_x, odom_y, odom_rot;
float deca_x, deca_y;
float tx=0, ty=0, theta=0;
float decagoal_x, decagoal_y;

// CMD_VEL Publisher
ros::Publisher cmd_publisher;
ros::Publisher backup_publisher;

float odom2decaX(float x, float y) {
    return x*cos(theta)-y*sin(theta)+tx;
}
float odom2decaY(float x, float y) {
    return x*sin(theta)+y*cos(theta)+ty;
}

float deca2OdomX(float x, float y){
    return (x-tx)*cos(theta)+(y-ty)*sin(theta);
}
float deca2OdomY(float x, float y){
    return -(x-tx)*sin(theta)+(y-ty)*cos(theta);
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
    double quatx= msg->pose.pose.orientation.x;
    double quaty= msg->pose.pose.orientation.y;
    double quatz= msg->pose.pose.orientation.z;
    double quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_rot = yaw;
	
}
void tagCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("grabbing tag");
    deca_x = msg->linear.x;
    deca_y = msg->linear.y;
}
void offsetCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    tx = msg->x;
    ty = msg->y;
    theta = msg->z;
    // update goal if navigating via decawave.
    if(cmd.is_deca == true){
        go_x = deca2OdomX(decagoal_x,decagoal_y);
        go_y = deca2OdomY(decagoal_x,decagoal_y);
    }
    
}
// This function determines the point the robot will travel to next. It runs each time the talker sends a point.
void updateLongPos(const camp_goto::Cmd::ConstPtr& msg){
    //ROS_INFO("grabbing destination");
    cmd.stop = msg->stop;
    cmd.is_relative = msg->is_relative;
    cmd.is_deca = msg->is_deca;

    if(cmd.is_deca == false){
        go_x = msg->destination.x;
        go_y = msg->destination.y;
        if(cmd.is_relative == true){
            //relative mode
            go_x += odom_x;
            go_y += odom_y;
        }
    } else {
        //convert decawave to odometry
        decagoal_x = msg->destination.x;
        decagoal_y = msg->destination.y;
        if(cmd.is_relative == true) {
            decagoal_x += deca_x;
            decagoal_y += deca_y;
        }
        go_x = deca2OdomX(decagoal_x,decagoal_y);
        go_y = deca2OdomY(decagoal_x,decagoal_y);
    }

    if (msg->speed >= 0.0 && msg->speed <= 1.0){
        param.speed = msg->speed;
    }
    if (msg->destination_stop_distance >= 0.0){
        param.dest_stop = msg->destination_stop_distance;
    }
    if (msg->emergency_stop_distance >= 0.0){
        param.emg_stop =  msg->emergency_stop_distance;
    }
    if (msg->emergency_stop_angle >= 0 && msg->emergency_stop_angle <= 180){
        param.emeg_stop_angle = msg->emergency_stop_angle;
    }
}

// This just checks if something is in front of the robot real fast.
float somethingInFront() {
    float closest_front_object;
    //set the front angle to 0 for positive direction, 180 for nagative direction
    int front_angle = 90-90*velocity_direction;
    if(ranges.size() == 360){
        // Find how close the closest object is (55 front scans)
        if (ranges.at(front_angle) != 0) closest_front_object = ranges.at(front_angle);
        else closest_front_object = 500;
        for(int i = 1; i < param.emeg_stop_angle; i++) {
            if(ranges.at(front_angle+i) < closest_front_object && ranges.at(front_angle+i) != 0) closest_front_object = ranges.at(front_angle+i);
            if(ranges.at(360-front_angle-i) < closest_front_object && ranges.at(360-front_angle-i) != 0) closest_front_object = ranges.at(360-front_angle-i);
        }
        return closest_front_object;
    } else {
        return 0.0;
    }
}
/**
 * This function navigates the robot to the point (go_x,go_y) based on the odometry.
 */
void pointToPoint() {
    // find the difference between the robot's actual heading and the heading it needs to get to the point.
    heading = odom_rot;
    float gx = go_x - odom_x;
    float gy = go_y - odom_y;
    float desired_heading = atan2(gy,gx);
    // also find how far from the goal it is
    go_distance = sqrt(gx*gx+gy*gy);
    
    // change any angles so that they are between -PI and PI
    float head_error = desired_heading - heading;
    while ((head_error > PI) || (head_error < 0-PI)) {
        if(head_error > PI) head_error = head_error - 2*PI;
        if(head_error < 0-PI) head_error = head_error + 2*PI;
    }

    // to implement backwards movement, change angles to be between -pi/2 and pi/2, and set velocity direction to -1 for corrected angles.
    velocity_direction = 1;
    if(head_error > PI/2) {
        head_error = head_error - PI;
        velocity_direction = -1;
    } 
    if(head_error < 0-PI/2) {
        head_error = head_error + PI;
        velocity_direction = -1;
    }
    //ROS_INFO("\ndes: %f\nhed: %f\nerr: %f",desired_heading,heading,head_error);
    
    // set rotational speed relative to the heading error and correct for max speeds.
    z_ang_vel = 2.0*param.speed * head_error; // 2 seems to be a good Kp in matlab sims
    if(z_ang_vel > BURGER_MAX_ANG_VEL*param.speed) z_ang_vel = BURGER_MAX_ANG_VEL*param.speed;
    if(z_ang_vel < -BURGER_MAX_ANG_VEL*param.speed) z_ang_vel = -BURGER_MAX_ANG_VEL*param.speed;

    // set forwrd/backwarty based on how accurate the heading is and the velocity direction
    //x_vel = BURGER_MAX_LIN_VEL*(0.5+0.5*(1-abs(head_error)/PI));
    x_vel = BURGER_MAX_LIN_VEL*param.speed*(-0.1+1.1*(1-abs(head_error)/PI))*velocity_direction; //this one should turn tighter. 
    if(go_distance < param.dest_stop) {
        //stop if at a final point
        // distance set in command message
        x_vel = 0; 
        z_ang_vel = 0;
    }
    last_x = odom_x;
    last_y = odom_y;
    ROS_INFO("Debug info:\n---Transform---\nX Offset: %2.3f\nY Offset: %2.3f\nTheta:    %1.4f (%3.1f)\n---Positions---\nOdometry Yaw: %1.4f\nOdometry Coords: (%2.3f,%2.3f)\nOdometry Goal:   (%2.3f,%2.3f)\nReal TOF Coords: (%2.3f,%2.3f)\nEstimated TOF:   (%2.3f,%2.3f)\nTOF Goal:        (%2.3f,%2.3f)\n---Commands---\nSpeed: %1.2f\nForward Velocity: %f\nAngular Velocity: %f\n",tx,ty,theta,theta/PI*180,odom_rot,odom_x,odom_y,go_x,go_y,deca_x,deca_y,odom2decaX(odom_x,odom_y),odom2decaY(odom_x,odom_y),decagoal_x,decagoal_y,x_vel,z_ang_vel);
}

void shutdown_robot(int sig)
{
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    cmd_publisher.publish(vel_msg);
    ros::shutdown();
}

// This is the main function.
// It contains some code to run once and a while loop for repeating actions.
int main(int argc, char **argv){
	ros::init(argc, argv, "camp_goto");
	ros::NodeHandle nh;	
	signal(SIGINT, shutdown_robot);
	ros::Rate loop_rate(10);

	ros::Subscriber scan_subscriber = nh.subscribe("scan", 10, scanCallback);
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber tag_subscriber = nh.subscribe("filtered", 10, tagCallback);
    ros::Subscriber offset_subscriber = nh.subscribe("transform", 10, offsetCallback);
    ros::Subscriber go_pos_subscriber = nh.subscribe("go_cmd", 10, updateLongPos);
	cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    backup_publisher = nh.advertise<std_msgs::Bool>("backup_state", 10);

    std_msgs::Bool backupMsg;
    bool backup = false;
    float reset = 0.0;
	while(ros::ok()) {
        //Determine motor instructions from current point and the go vars
        // pointToPoint();

        // determine if a backup is needed
        reset = somethingInFront();
        if (reset > param.emg_stop + 0.3) backup = false;
        if (reset < param.emg_stop) backup = true;
        //if no backu is needed
        if (backup == false) {
            //navigate as usual
            pointToPoint();
        // if a backup is needed
        } else {
            //reverse direction temporarily for the somethignInFront test
            velocity_direction = -velocity_direction;
            //look in the new direction for obstacles
            //if the path is clear,
            if (somethingInFront() > param.emg_stop){
                // fix velocity direction
                velocity_direction = -velocity_direction;
                //navigate as usual
                pointToPoint();
                // throw out forward direction and move backward in a straight line
                x_vel = -velocity_direction*BURGER_MAX_LIN_VEL*param.speed;
                //z_ang_vel = 0;
            //if the path is not clear
            } else {
                // fix velocity direction
                velocity_direction = -velocity_direction;
                //return to normal navigation
                pointToPoint();
                // don't move forward or back, to prevent infinite stuck periods
                x_vel = 0;
            }
        }
        
        if(cmd.stop == true){
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
        backupMsg.data = backup;
	    cmd_publisher.publish(vel_msg);
        backup_publisher.publish(backupMsg);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
	return 0;
}
