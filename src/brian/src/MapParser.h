#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

class MapParser
{
public:
    void callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
    {
        
    }

    bool dump()
    {
        return dump_tga();
    }

private:

    uint32_t _height, _width;

    vector<int8_t> _map_data;

    bool dump_tga()
    {

    }
};

#endif // MAP_PARSER_H