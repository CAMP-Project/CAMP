#include "MMArbitrator.h"

/**
 * @brief Construct a new MMArbitrator::MMArbitrator object
 * 
 * @param n ROS Nodehandle object
 */
MMArbitrator::MMArbitrator(ros::NodeHandle n) 
{
    // Setup client for the multimaster entry point
    this->_mm_entry_point = n.serviceClient<fkie_multimaster_msgs::DiscoverMasters>("master_discovery/list_masters");
    this->_master_state = n.subscribe("master_discovery/changes", 10, &MMArbitrator::master_state_callback, this);
    ros::spinOnce();

    
    // Subscribe to the current robot's map and odometry
    // this->_odom = n.subscribe();
    // this->_map_sub = n.subscribe();

    if (this->_mm_entry_point.call(this->_fkie_service))
    {
        // ROS_INFO("ROS Master 0: %s", srv.response.masters.at(0).name.c_str());
        int s = this->_fkie_service.response.masters.size();
        for (int i = 0; i < s; i++)
        {
            // Print name and push to available hosts vector
            ROS_INFO("ROS Master %d : %s", i, this->_fkie_service.response.masters.at(i).name.c_str());
            this->_available.push_back(this->_fkie_service.response.masters.at(i).name.c_str());
        }
    }

    // ROS_INFO("Masters: %s", response.masters.at(0).name.c_str());

}

/**
 * @brief Sync the current nodes with what is available on the network
 * 
 * @param n NodeHandle object to use to update the parameters of this class
 */
void MMArbitrator::sync(ros::NodeHandle n)
{
    
}

/**
 * @brief Callback function for the positions subscribers
 * 
 * @param foo Index of the object to update
 * @param pos Twist object
 */
void MMArbitrator::position_callback(int foo, const geometry_msgs::Twist::ConstPtr& pos)
{}

/**
 * @brief Callback function for the maps subscribers
 * 
 * @param foo Index of the object to update
 * @param map OccupancyGrid object
 */
void MMArbitrator::map_callback(int foo, const nav_msgs::OccupancyGrid::ConstPtr& map)
{}

/**
 * @brief Collect the name of the current master from the system
 * 
 * @param ms MasterState object
 */
void MMArbitrator::master_state_callback(const fkie_multimaster_msgs::MasterState::ConstPtr& ms)
{ this->_current_name = ms->master.name.c_str(); }

/**
 * @brief Class Destructor
 */
MMArbitrator::~MMArbitrator()
{
    this->_available.clear();
    this->_positions.clear();
    this->_maps.clear();
    
}
