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
    ros::spinOnce(); // call the master state callback function

    // Subscribe to the current robot's map and odometry
    this->_odom_sub = n.subscribe("odom", 10, &MMArbitrator::_position_callback, this);
    this->_map_sub = n.subscribe("map", 10, &MMArbitrator::_map_callback, this);
    
    // Sync up the list of available hosts to the what is 
    sync(n);

    ros::spinOnce();
}

/**
 * @brief Sync the current nodes with what is available on the network
 * 
 * @param n NodeHandle object to use to update the parameters of this class
 */
void MMArbitrator::sync(ros::NodeHandle n)
{
    ros::spinOnce();
    if (this->_mm_entry_point.call(this->_fkie_service))
    {
        int s = this->_fkie_service.response.masters.size();
        // If the number of hosts is the same between runs, no need to run everything again
        if (this->_available.size() == s)
            return;
        
        ROS_INFO("Response Size: %d", s);
        for (int i = 0; i < s; i++)
        {
            std::string new_host = this->_fkie_service.response.masters.at(i).name;
            // Check to see if the host acquired is already part of known hosts
            if(!(std::find(this->_available.begin(), this->_available.end(), new_host ) != this->_available.end()))
            {
                ROS_INFO("Host not found in list.");
                ROS_INFO("Adding ROS Master %d : %s", i, new_host);
                this->_available.push_back(new_host);
            }
            // If the current host is found, nothing else needs to be done and 
            // the loop can move onto the next element
        }
    }
    else
        ROS_INFO("Error calling master_discovery/list_masters service");

    // Update callback functions
    ros::spinOnce();
}

/**
 * @brief Callback function for the positions subscribers
 * 
 * @param foo Index of the object to update
 * @param pos Twist object
 */
void MMArbitrator::mm_position_callback(int foo, const geometry_msgs::Twist::ConstPtr& pos)
{}

/**
 * @brief Callback function for the local position
 * 
 * @param pos Constant pointer to a Twist object
 */
void MMArbitrator::_position_callback(const geometry_msgs::Twist::ConstPtr& pos)
{ 
    this->_my_odom.linear = pos->linear;
    this->_my_odom.angular = pos->angular;
}

/**
 * @brief Callback function for the maps subscribers
 * 
 * @param foo Index of the object to update
 * @param map OccupancyGrid object
 */
void MMArbitrator::mm_map_callback(int foo, const nav_msgs::OccupancyGrid::ConstPtr& map)
{}

/**
 * @brief Callback function for the local map
 * 
 * @param map 
 */
void MMArbitrator::_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{ 
    this->_my_map.data = map->data;
    this->_my_map.header = map->header;
    this->_my_map.info = map->info;
}

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
