/* 
This function should be run in additioon to other nodes to calibrate the odom/deca conversion.
Started 4/15/2021 by Tyler Pigott
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>
#define PI 3.14159265

using namespace std;

// Parameters for later
const int min_point_count = 10;
const float new_point_distance = 0.1;
const float bad_theta_threshold = 20;
const float bad_data_threshold = 5;
const float theta_found_threshold = 0.001;
const float d_theta = 0.0001;
float theta_0 = 0;

vector<float> odom_x, odom_y, deca_x, deca_y;
float odom_in_x,odom_in_y,deca_in_x,deca_in_y;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("grabbing odom");
    odom_in_x = msg->pose.pose.position.x;
    odom_in_y = msg->pose.pose.position.y;
}
void tagCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("grabbing tag");
    deca_in_x = msg->linear.x;
    deca_in_y = msg->linear.y;
}

// Mass coordinate vector commands
void pushCoords(){
    odom_x.push_back(odom_in_x);
    odom_y.push_back(odom_in_y);
    deca_x.push_back(deca_in_x);
    deca_y.push_back(deca_in_y);
}
void clearCoords(){
    odom_x.clear();
    odom_y.clear();
    deca_x.clear();
    deca_y.clear();
}

// The beefy part
void publishOffsets(ros::Publisher transform_publisher){
    // i think i said that odom is the input and deca is the output.
    geometry_msgs::Vector3 tr;
    float tx,ty,theta,last_theta,run;
    float a,b,a2,b2,tx2,ty2,n,f,f2,f_prime,e;

    theta = theta_0;
    last_theta = theta_0;
    run = 1;
    int attempt = 1;

    while(run == 1){
        run = 0;
        a = cos(theta);
        b = sin(theta);
        a2 = cos(theta + d_theta);
        b2 = sin(theta + d_theta);
        tx = 0;
        ty = 0;
        tx2 = 0;
        ty2 = 0;
        n = odom_x.size();
        for(int i = 0; i < n; i++){
            tx = tx + (1/n)*(deca_x[i]-a*odom_x[i]+b*odom_y[i]);
            ty = ty + (1/n)*(deca_y[i]-a*odom_y[i]-b*odom_x[i]);
            tx2 = tx2 + (1/n)*(deca_x[i]-a2*odom_x[i]+b2*odom_y[i]);
            ty2 = ty2 + (1/n)*(deca_y[i]-a2*odom_y[i]-b2*odom_x[i]);
        }
        f = 0;
        f2 = 0;
        for(int i = 0; i < n; i++){
            f = f + (ty-deca_y[i])*(a*odom_x[i]-b*odom_y[i]) - (tx-deca_x[i])*(b*odom_x[i]+a*odom_y[i]);
            f2 = f2 + (ty2-deca_y[i])*(a2*odom_x[i]-b2*odom_y[i]) - (tx2-deca_x[i])*(b2*odom_x[i]+a2*odom_y[i]);
        }
        f_prime = (f2 - f)/d_theta;
        last_theta = theta;
        theta = theta - f/f_prime;
        if (theta > PI || theta < -PI) theta = theta - floor(theta/(PI))*PI;
        if (abs((theta - last_theta)/theta) > theta_found_threshold) 
            run = 1; 
        else {
            e = 0;
            for(int i = 0; i < n; i++){
                e = e + pow((a*odom_x[i]-b*odom_y[i]+tx-deca_x[i]),2)+pow((b*odom_x[i]+a*odom_y[i]+ty-deca_y[i]),2);
            }
            ROS_INFO("-- Attempt: %d --",e);
            ROS_INFO("Error: %f",e);
            ROS_INFO("Theta: %f",theta);
            if(e > bad_theta_threshold) {
                theta = theta - theta/abs(theta)*PI;
                run = 1;
                attempt++;
                if(attempt > 3) return;

            }
            //else if(e > bad_data_threshold) do something that fixes it i guess? throw everything out?
        }

    }

    theta_0 = theta;

    tr.x = tx;
    tr.y = ty;
    tr.z = theta;
    ROS_INFO("pos offsets: (%f/%f)",tx,ty);
    ROS_INFO("Theta: %f",theta);
	transform_publisher.publish(tr);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle nh;	
	ros::Rate loop_rate(10);
    
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber tag_subscriber = nh.subscribe("filtered", 10, tagCallback);
	ros::Publisher transform_publisher = nh.advertise<geometry_msgs::Vector3>("transform", 10);
    
    while(ros::ok()) {

        // Do this if there are no point syet
        if(odom_x.size() == 0) {
            ROS_INFO("First Push");
            pushCoords();
            ROS_INFO("size: %d",odom_x.size());
        }

        // Do this if its time for another point
        if(sqrt(pow((odom_x.back() - odom_in_x),2) + pow((odom_y.back() - odom_in_y),2))>new_point_distance) {
            pushCoords();
            ROS_INFO("size: %d",odom_x.size());
        }

        // Do this if there are enough points to do a conversion
        if(odom_x.size() >= min_point_count){
            ROS_INFO("Publishing...");
            publishOffsets(transform_publisher);
            clearCoords();
            ROS_INFO("size: %d",odom_x.size());
        }

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}