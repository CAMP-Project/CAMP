#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <fstream>

class MapParser
{
public:

    /**
     * @brief Message callback to collect data from a ROS message
     * 
     * @param msg nav_msgs::OccupancyGrid::ConstPtr
     */
    void callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
    {
        this->_height = msg->info.height;
        this->_width = msg->info.width;

        this->_map_resolution = msg->info.resolution;

        this->_map_data = msg->data;

        this->_map_origin = msg->info.origin;
        
    }

    /**
     * @brief Build a 2D map and dump the map to a TGA file
     * 
     * @return true If operation completed successfully
     * @return false If operations completed unsuccessfully
     */
    bool dump()
    {
        _build_map();
        return _dump_tga();
    }

    ~MapParser()
    {
        this->_map_data.clear(); // Clear Map Data
    }

private:

    uint32_t _height, _width;

    float _map_resolution;

    std::vector<int8_t> _map_data;

    geometry_msgs::Pose _map_origin;

    typedef struct 
    {
        unsigned char red, green, blue;
    } RGB_t;
    

    std::vector<std::vector <RGB_t>> _output_map;

    /**
     * @brief Build a 2D Vector of the row-major ordered map data
     */
    void _build_map()
    {
        int k = 0; // Kepp track of position inside of the row-major ordered map data

        // Interate in 2D to build full map
        for (int i = 0; i < this->_output_map.size(); i++)
        {
            std::vector <RGB_t> foo;
            _output_map.push_back(foo);
            for (int j = 0; j < this->_output_map[i].size(); j++)
            {
                uint8_t probability_data = _map_data[k];
                RGB_t color;
                if (probability_data != -1) // If data is available
                {
                    color.red = 255 - (probability_data/100 * 255);
                    color.green = color.red;
                    color.blue = color.green;
                } 
                else // If no data is available output this wierd color
                {
                    color.red = 255;
                    color.green = 0;
                    color.blue = 255;
                }
                // Write color data to 2D vector
                _output_map[i].push_back(color);

                k++;
            }
        }
        
    }

    /**
     * @brief Output a TGA file
     * 
     * @return true If the file output was successful
     * @return false If the file output was uncessful
     */
    bool _dump_tga()
    {
        std::ofstream tgafile( "map.tga", std::ios::binary );
        if (!tgafile) return false; // Something bad happened

        // TGA Header
        unsigned char header [ 18 ] = { 0 };
        header[  2 ] = 1; // truecolor
        header[ 12 ] =  this->_width        & 0xFF;
        header[ 13 ] = (this->_width  >> 8) & 0xFF;
        header[ 14 ] =  this->_height       & 0xFF;
        header[ 15 ] = (this->_height >> 8) & 0xFF;
        header[ 16 ] = 24;  // bits per pixel

        tgafile.write((const char*) header, 18);

        // TGA color data 
        for (int i = 0; i < this->_height; i++)
        {
            for (int j = 0; j < this->_width; j++)
            {
                tgafile.put(_output_map[i][j].blue);
                tgafile.put(_output_map[i][j].green);
                tgafile.put(_output_map[i][j].red);
            }
        }
        
        // TGA Footer
        static const char footer[ 26 ] =
        "\0\0\0\0"  // no extension area
        "\0\0\0\0"  // no developer directory
        "TRUEVISION-XFILE"  // yep, this is a TGA file
        ".";
        tgafile.write( footer, 26 );

        tgafile.close();
        return true; // SUCCESS
    }
};

#endif // MAP_PARSER_H