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

    ros::ServiceClient client = n.serviceClient<fkie_multimaster_msgs::DiscoverMasters>("list_masters");

    fkie_multimaster_msgs::DiscoverMasters srv;
    if (client.call(srv))
        ROS_INFO("ROS Master 0: %s", srv.response.masters.at(0).name.c_str());
    

    // ROS_INFO("Masters: %s", response.masters.at(0).name.c_str());

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
