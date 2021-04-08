#include "MMArbitrator.h"

/**
 * @brief Construct a new MMArbitrator::MMArbitrator object
 * 
 * @param n ROS Nodehandle object
 */
MMArbitrator::MMArbitrator(ros::NodeHandle n) 
{
    // Subscribe to the multimaster entry point
    // this->_mm_entry_point = n.subscribe();

    // Subscribe to the current robot's map and odometry
    // this->_odom = n.subscribe();
    // this->_map_sub = n.subscribe();
}

/**
 * @brief Sync the current nodes with what is available on the network
 * 
 * @param n 
 */
void MMArbitrator::sync(ros::NodeHandle n)
{
    
}

void MMArbitrator::position_callback()
{}

void MMArbitrator::map_callback()
{}

void MMArbitrator::mm_callback()
{}

/**
 * @brief Class Destructor
 */
MMArbitrator::~MMArbitrator()
{
    
}
