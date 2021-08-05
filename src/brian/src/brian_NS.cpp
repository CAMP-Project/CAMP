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
#include <unistd.h>

#include "MapParser.h"
#include "MMArbitrator.h"

using namespace std;

float angle_min, angle_max, angle_increment, scan_time, range_min, range_max;
vector<float> ranges, intensities;

MapParser mapParser;

bool data_avail = false;

/**
 * @brief Callback function for the Lidar data
 * 
 * @param msg sensor_msgs::LaserScan pointer
 */
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

/**
 * @brief Callback function for reading the Map from the SLAM algorithm
 * 
 * @param msg nav_msgs::OccupancyGrid pointer
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
        mapParser.height = msg->info.height;
        mapParser.width = msg->info.width;

        mapParser.map_resolution = msg->info.resolution;

        mapParser.map_data = msg->data;

        mapParser.map_origin = msg->info.origin;

        data_avail = true;

        // printf("%d\n", suc);
        printf("Data Read\n");
}

int main(int argc, char **argv)
{

    // To avoid cancelling arbitrators between cores, grab the hostname of the system and 
    // attach it to the "brian" keyword. 
    char hostname[_SC_HOST_NAME_MAX];
    int result;
    result = gethostname(hostname, HOST_NAME_MAX);

    // Concatonate the string "brian" with the character array "hostname".
    std::string node_name = std::string("brian_") + hostname;

    // For every character in the new node name, verify that it does not contain any illegal characters
    // native to ROS. If such illegal characters exist, replace them with an underscore.
    string illegalChars = "\\/:?\"<>|~`';-+=!@#$%^&*()*";
    for (int i = 0; i < node_name.length(); i++) {
        bool foundIllegalChar = illegalChars.find(node_name[i]) != string::npos;
        if (foundIllegalChar) {
            node_name[i] = '_';
        }
    }

    // Check if the name has been created successfully.
    ROS_INFO("Hostname: %s", node_name.c_str());

    // Create a new node with the concatonated node name.
    ros::init(argc, argv, node_name);

    // geometry_msgs::Twist vel_msg;
    
    ros::NodeHandle n;
    ros::Rate r(10);

    // Launch the Data Arbitrator
    ROS_INFO("Looking for other Cores");
    MMArbitrator mma(n);

    // Map Parser Code to output the current map into an image file on the current system's disk
    // ros::Subscriber lidar = n.subscribe("scan", 100, msgCallback);

    // ros::Subscriber map_sub = n.subscribe("map", 100, mapCallback);

    // MapParser mapParser;

    while(ros::ok())
    {
        mma.sync(n);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}