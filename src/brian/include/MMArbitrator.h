#ifndef MM_ARBITRATOR_H
#define MM_ARBITRATOR_H

/*
 * This header files holds the class for the Multi Master Arbitrator
 * The Multi Master Arbitrator is responsible for discovering additional
 * robots on the ROS network and sync up the various nodes with the current
 * robot.
 */ 

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <vector>

#include "fkie_multimaster_msgs/DiscoverMasters.h"

class MMArbitrator
{
public:
    MMArbitrator(ros::NodeHandle);

    void sync(ros::NodeHandle);

    // void 

    ~MMArbitrator();
private:
    // Vectors to hold all the subsribers and publishers
    // that are dynamically added to the system

    std::vector<ros::Subscriber *> _subs;
    std::vector<ros::Publisher *> _pubs;

    // Multimaster entry point, this is how this system
    // finds other robots on the network
    ros::Subscriber _mm_entry_point;

    // Subscriber to the current robot odometry to later publish
    ros::Subscriber _odom;

    // Subscribe to the current robot map to later publish
    ros::Subscriber _map_sub;

    // Odometry sub callback 
    void position_callback();

    // Map sub callback
    void map_callback();

    // Multimaster callback
    void mm_callback();
    
};

#endif // MM_ARBITRATOR_H