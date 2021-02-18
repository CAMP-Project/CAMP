#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <fstream>
#include <iostream>

class MapParser
{
public:

    uint32_t height, width;

    float map_resolution;

    std::vector<int8_t> map_data;

    geometry_msgs::Pose map_origin;

    // /**
    //  * @brief Message callback to collect data from a ROS message
    //  * 
    //  * @param msg nav_msgs::OccupancyGrid::ConstPtr
    //  */
    // static void callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
    // {
    //     _height = msg->info.height;
    //     _width = msg->info.width;

    //     _map_resolution = msg->info.resolution;

    //     _map_data = msg->data;

    //     _map_origin = msg->info.origin;
        
    // }

    /**
     * @brief Build a 2D map and dump the map to a TGA file
     * 
     * @return true If operation completed successfully
     * @return false If operations completed unsuccessfully
     */
    bool dump()
    {
        _build_map();
        return _pgm_dump();
    }

    ~MapParser()
    {
        // dump();
        this->map_data.clear(); // Clear Map Data
        _map_sub.shutdown();
    }

private:

    // static uint32_t _height, _width;

    // static float _map_resolution;

    // static std::vector<int8_t> _map_data;

    // static geometry_msgs::Pose _map_origin;

    typedef struct 
    {
        unsigned char red, green, blue;
    } RGB_t;
    

    std::vector<std::vector <RGB_t>> _output_map;

    ros::Subscriber _map_sub;

    /**
     * @brief Build a 2D Vector of the row-major ordered map data
     */
    void _build_map()
    {
        std::cout << "Building Map" << std::endl;
        int k = 0; // Kepp track of position inside of the row-major ordered map data

        // Interate in 2D to build full map
        for (int i = 0; i < this->height; i++)
        {
            std::vector <RGB_t> foo;
            _output_map.push_back(foo);
            for (int j = 0; j < this->width; j++)
            {
                uint8_t probability_data = map_data[k];
                std::cout << (int)probability_data;
                RGB_t color;
                if (probability_data != -1) // If data is available
                {
                    color.red = probability_data;
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
        std::cout << std::endl;
    }

    /**
     * @brief Output a TGA file
     * 
     * @return true If the file output was successful
     * @return false If the file output was uncessful
     */
    bool _dump_tga()
    {
        std::cout << "Taking dump into map.tga" << std::endl;
        std::ofstream tgafile( "map.tga", std::ios::binary );
        std::cout << "file opened" << std::endl;
        if (!tgafile) return false; // Something bad happened

        // TGA Header
        unsigned char header [ 18 ] = { 0 };
        header[  2 ] = 1; // truecolor
        header[ 12 ] =  width        & 0xFF;
        header[ 13 ] = (width  >> 8) & 0xFF;
        header[ 14 ] =  height       & 0xFF;
        header[ 15 ] = (height >> 8) & 0xFF;
        header[ 16 ] = 8;  // bits per pixel
        std::cout << "Writing Header" << std::endl;
        tgafile.write((const char*) header, 18);

        std::cout << "Writing data" << std::endl;
        // TGA color data 
        for (int i = _output_map.size() - 1; i >= 0; i--)
        {
            for (int j = 0; j < _output_map[i].size(); j++)
            {
                tgafile.put((char)_output_map[i][j].blue);
                tgafile.put((char)_output_map[i][j].green);
                tgafile.put((char)_output_map[i][j].red);
                // tgafile.put((char)255);
                // tgafile.put((char)0);
                // tgafile.put((char)255);
            }
        }
        std::cout << "Writing Footer" << std::endl;
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

    bool _pgm_dump()
    {
        std::ofstream pgmfile( "map.pgm", std::ios::binary );
        if (!pgmfile) return false;

        pgmfile << 'P' << '2' << std::endl;
        pgmfile << width << ' ' << height << std::endl;

        for (int i = 0; i < _output_map.size(); i++)
        {
            for (int j = 0; j < _output_map[i].size(); j++)
            {
                pgmfile << (int)_output_map[i][j].blue << ' ';
            }
            pgmfile << std::endl;
        }
        return true;
    }
};

#endif // MAP_PARSER_H