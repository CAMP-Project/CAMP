#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <vector>
#include <map>

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
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "brian");

    ros::NodeHandle n;
    ros::Subscriber lidar = n.subscribe("scan", 100, msgCallback);

    ros::spin();
    // cout << "Hello, I am Brian" << endl;
    return 0;
}
