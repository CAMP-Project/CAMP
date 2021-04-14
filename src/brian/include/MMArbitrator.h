#ifndef MM_ARBITRATOR_H
#define MM_ARBITRATOR_H

/*
 * This header files holds the class for the Multi Master Arbitrator
 * The Multi Master Arbitrator is responsible for discovering additional
 * robots on the ROS network and sync up the various nodes with the current
 * robot.
 */ 

// ROS Objects
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/service_client.h>

// ROS Messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

// Standard C/C++ Libraries
#include <vector>
#include <string>
#include <boost/bind.hpp>

// Development Libraries from within this project
#include "fkie_multimaster_msgs/DiscoverMasters.h"
#include "fkie_multimaster_msgs/MasterState.h"

/**
 * @file MMArbitator.h
 * @brief Arbitrator object to publish and subsribe to new data available on the network
 * @author Manpreet Singh (manpreet-singh)
 */
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
    ros::ServiceClient _mm_entry_point;
    // DiscoverMasters service data structure
    fkie_multimaster_msgs::DiscoverMasters _fkie_service;

    // Information about the system running this node
    ros::Subscriber _master_state;

    // Subscriber to the current robot odometry to later publish
    ros::Subscriber _odom;

    // Subscribe to the current robot map to later publish
    ros::Subscriber _map_sub;

    // List of other hosts on the network
    std::vector<std::string> _available;

    // List of positions of all hosts on the network
    std::vector<geometry_msgs::Twist> _positions;

    // List of maps of all hosts on the network 
    std::vector<nav_msgs::OccupancyGrid> _maps;

    std::string _current_name = "";

    // Odometry sub callback 

    void position_callback(int foo, const geometry_msgs::Twist::ConstPtr& pos);

    // Map sub callback

    void map_callback(int foo, const nav_msgs::OccupancyGrid::ConstPtr& map);

    // MasterState callback

    void master_state_callback(const fkie_multimaster_msgs::MasterState::ConstPtr& ms);
};

#endif // MM_ARBITRATOR_H