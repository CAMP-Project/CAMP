#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Point.h>
#include "camp_multiagent/Robot.h"

camp_multiagent::Robot robot;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot.position = msg->pose.pose.position;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    robot.info = msg->info;
    robot.data = msg->data;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "multiagent_packager");
	ros::NodeHandle nh;	
    //loop every 10 secs
	ros::Rate loop_rate(0.1);
    
    ros::Subscriber position_subscriber = nh.subscribe("odom", 10, positionCallback);
    ros::Subscriber map_subscriber = nh.subscribe("map", 10, mapCallback);
	ros::Publisher robot_publisher = nh.advertise<camp_multiagent::Robot>("robot", 10);

    while(ros::ok()) {
        robot.header.stamp = ros::Time::now();
        robot.header.frame_id = "deca";

	    robot_publisher.publish(robot);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
}
