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

    // Subscribe to the current robot's map and odometry
    this->_odom_sub = n.subscribe("odom", 10, &MMArbitrator::_position_callback, this);
    this->_map_sub = n.subscribe("map", 10, &MMArbitrator::_map_callback, this);

    // Sync up the list of available hosts to the rest of the system
    sync(n);
}

/**
 * @brief Sync the current node with what is available on the network
 * 
 * @param n NodeHandle object to use to update the parameters of this class
 */
void MMArbitrator::sync(ros::NodeHandle n)
{
    ros::spinOnce();
    
    // If we don't know who we are, no point in going forward
    if (this->_current_name.empty())
        return;

    if (this->_mm_entry_point.call(this->_fkie_service))
    {
        int s = this->_fkie_service.response.masters.size();
        // If the number of hosts is the same between runs, no need to run everything again
        if (this->_available.size() == s-1)
        {
            debug_print("Sync");
            // Advertise the current robot's map and odometry under a different name
            // ROS_INFO("%d ~ %d", _map_pub.getTopic().empty(), _current_name.size());
            if (this->_current_name.size() > 0 && _map_pub.getTopic().empty())
            {
                ROS_INFO("Setting up my publishers");
                std::string new_odom_name = this->_current_name;
                new_odom_name.append("/odom");
                std::string new_map_name = this->_current_name;
                new_map_name.append("/map");
                this->_odom_pub = n.advertise<geometry_msgs::Twist>(new_odom_name, 10);
                this->_map_pub = n.advertise<nav_msgs::OccupancyGrid>(new_map_name, 10);
            }
            // Lets publish the current robot's information before grabbing other things
            if (this->_current_name.size() > 2)
            {
                this->_map_pub.publish(this->_my_map);
                this->_odom_pub.publish(this->_my_odom);
            }
            return;
        }
        // Who am i?
        debug_print("I AM: "+_current_name);
        ROS_INFO("Response Size: %d", s);
        for (int i = 0; i < s; i++)
        {
            // Name of "new" host
            std::string new_host = this->_fkie_service.response.masters.at(i).name;
            debug_print("Checkhost: "+new_host);
            // Remove illegal characters from new host name
            new_host.erase(std::remove_if(new_host.begin(), new_host.end(),
                            [](const char c)
                            {return c=='.';}), new_host.end());

            //Check is the new host name is number and convert to something better
            if (is_number(new_host))
                new_host = this->shift_num_to_char(new_host);

            // No need to subscribe to our own nodes
            if (new_host.compare(this->_current_name) == 0)
            {
                debug_print("BAILING: "+new_host);
                continue;
            }

            // Check to see if the host acquired is already part of known hosts
            if(std::find(this->_available.begin(), this->_available.end(), new_host ) == this->_available.end())
            {
                ROS_INFO("Host not found in list.");
                ROS_INFO("Adding ROS Master %d : %s", i, new_host.c_str());
                this->_available.push_back(new_host);

                // Check if subscribers exist for this object
                ros::V_Subscriber::iterator ch = std::find_if(this->_subs.begin(), this->_subs.end(), boost::bind(&MMArbitrator::_sub_comp, this, _1, new_host+"/odom" ) );
                if (ch == this->_subs.end() && new_host != this->_current_name)
                {
                    // Odom Subscriber for this host has not been setup yet. Let's set it up now.
                    ros::Subscriber new_odom = n.subscribe<geometry_msgs::Twist>(new_host+"/odom", 10, boost::bind(&MMArbitrator::mm_position_callback, this, (int)this->_subs.size(), _1));
                    this->_subs.push_back(new_odom);
                    ros::Subscriber new_map = n.subscribe<nav_msgs::OccupancyGrid>(new_host+"/map", 10, boost::bind(&MMArbitrator::mm_map_callback, this, (int)this->_subs.size(), _1));
                    this->_subs.push_back(new_map);
                }
            }
            // If the current host is found, nothing else needs to be done and 
            // the loop can move onto the next element
        }
    }
    else
        ROS_INFO("Error calling master_discovery/list_masters service");

    #ifdef DEBUG
        std::string hosts;
        for (int i = 0; i < this->_available.size(); i++)
            hosts.append(_available.at(i)+" ");
        
        ROS_INFO("Available hosts: %s", hosts.c_str());
                
    #endif
    ROS_INFO("End of initial sync");
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
{
    // Initialize the current name string if it is empty, no need to update it everytime this class is called.
    this->_my_state.master = ms->master;
    this->_my_state.state = ms->state;
    ROS_INFO("Somebody called the master state callback!!!");
    if (this->_current_name.empty())
    {
        this->_current_name = ms->master.name;
        this->_current_name.erase(std::remove_if(this->_current_name.begin(), this->_current_name.end(),
                                [](const char c)
                                {return c=='.';}), this->_current_name.end());
    }
}

/**
 * @brief Compare the subsriber with a topic name
 * 
 * @param obj Subscriber object to compare
 * @param name Name of a topic to compare
 * @return true The subscriber objecct has the same topic name as the name string passed into the function
 * @return false The topic name of the subscriber doesn't not match the string passed into the function
 */
bool MMArbitrator::_sub_comp(ros::Subscriber & obj, std::string name)
{
    if (obj.getTopic().compare(name) == 0)
        return true;
    else
        return false;
}

/**
 * @brief Class Destructor
 */
MMArbitrator::~MMArbitrator()
{
    this->_available.clear();
    this->_positions.clear();
    this->_maps.clear();
}

bool MMArbitrator::is_number(std::string s)
{
    for (int i = 0; i < s.length(); i++)
        if(isdigit(s[i]) == false)
            return false;
    
    return true;
}

std::string MMArbitrator::shift_num_to_char(std::string s)
{
    // lets do some magic 
    for (int i = 0; i < s.length(); i++)
        s[i] +=  17;

    return s;    
}

void MMArbitrator::debug_print(std::string s)
{
    #ifdef DEBUG
        ROS_INFO("DEBUG: %s", s.c_str());
    #endif
}