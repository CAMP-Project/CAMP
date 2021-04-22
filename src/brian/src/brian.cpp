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

    // for (int i = 0; i < (int)(angle_max/angle_increment); i++)
    // {
    //     printf("%f", ranges.at(i));
    // }
    // printf("\n");
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
        mapParser.height = msg->info.height;
        mapParser.width = msg->info.width;

        mapParser.map_resolution = msg->info.resolution;

        mapParser.map_data = msg->data;

        mapParser.map_origin = msg->info.origin;

        // bool suc = mapParser.dump();

        data_avail = true;

        // printf("%d\n", suc);
        printf("Data Read\n");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "brian");

    // geometry_msgs::Twist vel_msg;
    
    ros::NodeHandle n;
    ros::Rate r(10);

    ROS_INFO("Looking for other Cores");
    MMArbitrator mma(n);

    // ros::Subscriber lidar = n.subscribe("scan", 100, msgCallback);

    // ros::Subscriber map_sub = n.subscribe("map", 100, mapCallback);

    // MapParser mapParser;

    while(ros::ok())
    {
        // printf("Data: %d\n", data_avail);
        
        // if (data_avail)
        // {
        //     data_avail = false;
        //     printf("Height: %d \tWidth: %d\n", mapParser.height, mapParser.width);
        //     if (mapParser.height > 0 && mapParser.width > 0)
        //     {
        //         mapParser.dump();
        //         return 0;
        //     }
        // }

        mma.sync(n);

        ROS_INFO("sync success");

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("How did we get here?!?!");

    // ros::spin();

    // cout << "Height: " << mapParser.height << "\t Width: " << mapParser.width << endl;
    // cout << "Hello, I am Brian" << endl;

    return 0;
}
