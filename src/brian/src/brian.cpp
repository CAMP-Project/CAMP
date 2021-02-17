#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <vector>
#include <map>

#include "brian/MapParser.h"

using namespace std;

float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;

void msgCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    scan_time = msg->scan_time;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;
    intensities = msg->intensities;

    for (int i = 0; i < (int)(angle_max/angle_increment); i++)
    {
        printf("%f", ranges.at(i));
    }
    printf("\n");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "brian");

    geometry_msgs::Twist vel_msg;
    
    ros::NodeHandle n;
    ros::Subscriber lidar = n.subscribe("scan", 100, msgCallback);

    ros::spin();
    // cout << "Hello, I am Brian" << endl;
    return 0;
}
