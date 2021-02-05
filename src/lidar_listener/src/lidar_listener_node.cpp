/********************************************
 * The functionality in this package is		*
 * moving to the roomba_navigation package	*
 * for further development.					*
 ********************************************/
// ROS Default Header File
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>
using namespace std;
// MsgTutorial Message File Header
// The header file is automatically created when building the package.

// Message callback function. This is a function is called when a topic
// message named 'ros_tutorial_msg' is received. As an input message,
// the 'MsgTutorial' message of the 'ros_tutorials_topic' package is received.

float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;

float vel = 0, spin = 0, frontDistance;
int nuberoftimes;

void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    scan_time = msg->scan_time;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;
    intensities = msg->intensities;
	frontDistance = ranges.at(0);
	ROS_INFO("how far? = %f", frontDistance);
}

int main(int argc, char **argv){			// Node Main Function
	ros::init(argc, argv, "lidar_listener");	// Initializes Node Name
	ros::NodeHandle nh;				// Node handle declaration for communication with ROS system

	geometry_msgs::Twist vel_msg; //Make a variable to write the command data to.

	// Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
	ros::Rate loop_rate(10);

	// Lidar_listener subscribes to the lidar's scan, and cmd_publisher publishes a twist and the size of the publisher queue is set to 10.
	ros::Subscriber lidar_listener = nh.subscribe("scan", 10, msgCallback);
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
	nuberoftimes = 0;

	while(ros::ok()) {
		if(frontDistance > 0.2 || frontDistance == 0) vel = 0.1;
		else vel = 0;

		vel_msg.linear.x = vel;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = spin;
		
		//ROS_INFO("speed= %f", vel);
		
		cmd_publisher.publish(vel_msg);	//Publishes 'msg' message
		loop_rate.sleep();		// Goes to sleep according to the loop rate defined above.
		ros::spinOnce();		//Check for new messages.
		
		nuberoftimes++;
	}
	return 0;
}