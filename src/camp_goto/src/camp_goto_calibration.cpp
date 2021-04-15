/* 
This function should be run in additioon to other nodes to calibrate the odom/deca conversion.
Started 4/15/2021 by Tyler Pigott
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

using namespace std;

vector<float> odom_x, odom_y, deca_x, deca_y;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("grabbing odom");
    odom_x.push_back(msg->pose.pose.position.x);
    odom_y.push_back(msg->pose.pose.position.y);
}
void tagCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("grabbing tag");
    deca_x.push_back(msg->linear.x);
    deca_y.push_back(msg->linear.y);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle nh;	
    //camp_goto::transform tr;
	ros::Rate loop_rate(1);
    
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber tag_subscriber = nh.subscribe("filtered", 10, tagCallback);
	//ros::Publisher transform_publisher = nh.advertise<camp_goto::transform>("transform", 10);
    
    while(ros::ok()) {
        
        // I bet something real fancy goes here

	    //transform_publisher.publish(tr);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}