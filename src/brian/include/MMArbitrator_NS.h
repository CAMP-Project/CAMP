#ifndef MM_ARBITRATOR_NS_H
#define MM_ARBITRATOR_NS_H

/*
 * This header files holds the class for the Multi Master Arbitrator
 * The Multi Master Arbitrator is responsible for discovering additional
 * robots on the ROS network and sync up the various nodes with the current
 * robot.
 */ 

// ROS Objects
#include <ros/ros.h>

// ROS Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Standard C/C++ Libraries
#include <vector>
#include <string>
#include <algorithm>
#include <utility>
#include <boost/bind.hpp>

// Development Libraries from within this project as defined in CMakeLists.txt
#include "fkie_multimaster_msgs/DiscoverMasters.h"
#include "fkie_multimaster_msgs/MasterState.h"

// Service headers
#include "brian/RobotMapService.h"
#include "brian/RobotPositionService.h"

// Custom Messages
#include "brian/RobotKeyList.h"
#include "brian/OdometryLabeled.h"
#include "brian/OccupancyMapLabeled.h"
#include "brian/DecawaveLabeled.h"
#include "brian/DecawaveMapLabeled.h"

// Preprocessor Definitions
#define DEBUG 1

/**
 * @file MMArbitator.h
 * @brief Arbitrator object to publish and subsribe to new data available on the network
 * @author Manpreet Singh (manpreet-singh)
 */
class MMArbitrator_NS
{
public:
    MMArbitrator_NS(ros::NodeHandle);

    void sync(ros::NodeHandle);

    bool mapService(brian::RobotMapService::Request&, brian::RobotMapService::Response&);

    bool positionService(brian::RobotPositionService::Request&, brian::RobotPositionService::Response&);

    ~MMArbitrator_NS();
private:
    // Map to hold all the subsribers
    // that are dynamically added to the system
    std::map<std::string, ros::Subscriber> _sub_map;

    // Multimaster entry point, this is how this system
    // finds other robots on the network
    ros::ServiceClient _mm_entry_point;

    // DiscoverMasters service data structure
    fkie_multimaster_msgs::DiscoverMasters _fkie_service;

    // Information about the system running this node
    ros::Subscriber _master_state;

    //-------------------------------------------------------------------------------------------
    // *** SUBSCRIBERS & PUBLISHERS: ***
    // Subscribe to the current robot odometry to later publish
    ros::Subscriber _odom_sub;
    ros::Publisher _odom_pub;

    // Publisher for a labeled version of the robot's odometry
    ros::Publisher _odom_label_pub;

    // Subscribe to the current robot map to later publish
    ros::Subscriber _map_sub;
    ros::Publisher _map_pub;

    // Publisher for a labeled version of the robot's occupancy grid
    ros::Publisher _map_label_pub;

    // Subscribe to the current robot's decawave pose to later publish
    ros::Subscriber _deca_label_sub;
    ros::Publisher _deca_label_pub;

    // Subscribe to the current robot deca-map to later publish
    ros::Subscriber _deca_map_label_sub;
    ros::Publisher _deca_map_label_pub;

    // Publisher for the MMA-generated hostlist
    ros::Publisher _list_pub;
    //-------------------------------------------------------------------------------------------

    // List of other hosts on the network
    std::vector<std::string> _available;

    // List of positions of all hosts on the network
    std::map<std::string, nav_msgs::Odometry> _positions_map;

    // What a name ay?
    std::map<std::string, nav_msgs::OccupancyGrid> _maps_map;

    // Information of the current host that is running this code
    fkie_multimaster_msgs::MasterState _my_state;
    std::string _current_name;

    // List of all the current hosts obtained by the arbitrator on the current network.
    brian::RobotKeyList _host_list;

    //-------------------------------------------------------------------------------------------
    // *** DECAWAVE/ODOM MAPS/POSITIONS

    // Variables to hold on to the callback outputs.
    nav_msgs::Odometry _my_odom;
    nav_msgs::OccupancyGrid _my_map;
    geometry_msgs::Twist _my_deca;
    nav_msgs::OccupancyGrid _deca_map;

    // Variables to hold on to a labeled version of odometry and decawave for the current bot.
    brian::OdometryLabeled _odom_labeled;
    brian::DecawaveLabeled _deca_labeled;

    // Variables to hold on to a labeled version of the odom-based occupancy grid for the current bot
    // as well as the transformed deca-based occupancy grid for the current bot.
    brian::OccupancyMapLabeled _odom_map_labeled;
    brian::DecawaveMapLabeled _deca_map_labeled;
    //-------------------------------------------------------------------------------------------

    // ROS Service servers
    ros::ServiceServer _map_service_server;
    ros::ServiceServer _pos_service_server;

    // TODO: Update callback functions to map host name to data

    // Odometry sub callback for multimaster
    void mm_position_callback(std::string, const nav_msgs::Odometry::ConstPtr&);

    // Odometry sub callback for local odometry
    void _odom_position_callback(const nav_msgs::Odometry::ConstPtr&);

    // Twist sub callback for deca position
    void _deca_position_callback(const geometry_msgs::Twist::ConstPtr&);

    // Map sub callback for multimaster
    void mm_map_callback(std::string, const nav_msgs::OccupancyGrid::ConstPtr&);

    // Map sub callback for local map
    void _odom_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);

    // Map sub callback for deca map
    void _deca_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&); 

    // MasterState callback
    void master_state_callback(const fkie_multimaster_msgs::MasterState::ConstPtr&);

    // Compare a subscriber object to a string
    bool _sub_comp(ros::Subscriber&, std::string);

    // Compare a ROSMaster object to a string
    bool _host_comp(fkie_multimaster_msgs::ROSMaster, std::string);

    // Check if a string contains numbers
    bool is_number(std::string);

    // Shift numbers to characters and return as string
    std::string shift_num_to_char(std::string);

    void debug_print(std::string);

    void print_current_hosts();

    void update_host_list();

    std::string check_host(std::string);
};

#endif // MM_ARBITRATOR_NS_H