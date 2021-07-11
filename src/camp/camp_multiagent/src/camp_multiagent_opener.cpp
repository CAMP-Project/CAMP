#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "camp_multiagent/Robot.h"
#include <vector>
#include <math.h>
using namespace std;

geometry_msgs::Point decaPosition;
nav_msgs::OccupancyGrid decaMap;
struct transformOffsets{
    float x;
    float y;
    float theta;
} tf;

float odom2decaX(float x, float y) {
    return x*cos(tf.theta)-y*sin(tf.theta)+tf.x;
}
float odom2decaY(float x, float y) {
    return x*sin(tf.theta)+y*cos(tf.theta)+tf.y;
}

float deca2odomX(float x, float y){
    return (x-tf.x)*cos(tf.theta)+(y-tf.y)*sin(tf.theta);
}
float deca2odomY(float x, float y){
    return -(x-tf.x)*sin(tf.theta)+(y-tf.y)*cos(tf.theta);
}

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    decaPosition = msg->point;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    //ROS_INFO("starterddning tho read the map");
    decaMap.data = msg->data;
    decaMap.header = msg->header;
    decaMap.info = msg->info;
    //ROS_INFO("doen readin de map");
}

void transformCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    //this should still work
    tf.x = msg->x;
    tf.y = msg->y;
    tf.theta = msg->z;
}

geometry_msgs::Point getOdomPosition(){
    geometry_msgs::Point out;
    out.x = deca2odomX(decaPosition.x,decaPosition.y);
    out.y = deca2odomY(decaPosition.x,decaPosition.y);
    out.z = decaPosition.z;
    return out;
}

nav_msgs::OccupancyGrid getOdomMap(){
    nav_msgs::OccupancyGrid out;
    std::vector<signed char> dataIn, dataOut;
    int size;
    int half_size;
    int mIn, nIn;
    float resolution;

    //Get a bunch of map data
    dataIn = decaMap.data;
    size = decaMap.info.width;
    resolution = decaMap.info.resolution;

    // go through each element in the new array and fill it in with something
    half_size = size/2;
    //with maps centers alligned, take every element of our new decawave map and find what value on the odom map correlates.
    for(int m = 0; m < size; m++){
        for(int n = 0; n < size; n++){
            mIn = int(round(odom2decaX(m-half_size,n-half_size)+half_size));
            nIn = int(round(odom2decaY(m-half_size,n-half_size)+half_size));
            if (mIn >= 0 && mIn < size && nIn >= 0 && nIn < size) {
                dataOut.push_back(dataIn.at((mIn)*size+(nIn)));
            } else {
                dataOut.push_back(-1);
            }
            //ROS_INFO("makin map %d,%d",m,n);
        }
    }
    out.data = dataOut;

    out.info.resolution = decaMap.info.resolution;
    out.info.height = size;
    out.info.width = size;
    out.info.map_load_time = decaMap.info.map_load_time;
    out.info.origin.orientation = decaMap.info.origin.orientation;
    geometry_msgs::Point position;

    float tempX = (odom2decaX(0-half_size,0-half_size)+half_size)*out.info.resolution + decaMap.info.origin.position.x;
    float tempY = (odom2decaY(0-half_size,0-half_size)+half_size)*out.info.resolution + decaMap.info.origin.position.y;

    out.info.origin.position.x = deca2odomX(tempX,tempY);
    out.info.origin.position.y = deca2odomY(tempX,tempY);

    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";

    return out;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "multiagent_opener");
	ros::NodeHandle nh;	
    //loop every 10 secs
	ros::Rate loop_rate(0.1);
    
    ros::Subscriber position_subscriber = nh.subscribe("decapos", 10, positionCallback);
    ros::Subscriber map_subscriber = nh.subscribe("decamap", 10, mapCallback);
    ros::Subscriber transform_subscriber = nh.subscribe("transform", 10, transformCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("othermap",10);
    ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("otherpos",10);

    nav_msgs::OccupancyGrid map;
    geometry_msgs::PointStamped pos;


    while(ros::ok()) {
        pos.header.stamp = ros::Time::now();
        pos.header.frame_id = "map";
        pos.point = getOdomPosition();

        map = getOdomMap();

	    map_publisher.publish(map);
	    pos_publisher.publish(pos);
        // Check for new messages from subscribed nodes
		ros::spinOnce();
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // ROS_INFO("I SUPNNNED");
	}
}
