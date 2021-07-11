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

geometry_msgs::Point odomPosition;
nav_msgs::OccupancyGrid odomMap;
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

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odomPosition = msg->pose.pose.position;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    //ROS_INFO("starterddning tho read the map");
    odomMap.data = msg->data;
    odomMap.header = msg->header;
    odomMap.info = msg->info;
    //ROS_INFO("doen readin de map");
}

void transformCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    tf.x = msg->x;
    tf.y = msg->y;
    tf.theta = msg->z;
}

geometry_msgs::Point getDecaPosition(){
    geometry_msgs::Point out;
    out.x = odom2decaX(odomPosition.x,odomPosition.y);
    out.y = odom2decaY(odomPosition.x,odomPosition.y);
    out.z = odomPosition.z;
    return out;
}

nav_msgs::OccupancyGrid getDecaMap(){
    nav_msgs::OccupancyGrid out;
    std::vector<signed char> dataIn, dataOut;
    int width, height, newDimensions;
    int w2,h2,d2;
    int mIn, nIn;
    float resolution;

    //Get a bunch of map data
    dataIn = odomMap.data;
    width = odomMap.info.width;
    height = odomMap.info.height;
    resolution = odomMap.info.resolution;

    //Decide how big the new map is.
    //the old map can't be wider than it's diagonal at any rotation, so we will use that as our new width and height.
    newDimensions = sqrt(height*height+width*width);

    // go through each element in the new array and fill it in with something
    w2 = width/2;
    h2 = height/2;
    d2 = newDimensions/2;
    //with maps centers alligned, take every element of our new decawave map and find what value on the odom map correlates.
    for(int m = 0; m < newDimensions; m++){
        for(int n = 0; n < newDimensions; n++){
            mIn = int(round(deca2odomX(m-d2,n-d2)+h2));
            nIn = int(round(deca2odomY(m-d2,n-d2)+w2));
            if (mIn >= 0 && mIn < height && nIn >= 0 && nIn < width) {
                dataOut.push_back(dataIn.at((mIn)*height+(nIn)));
            } else {
                dataOut.push_back(-1);
            }
            //ROS_INFO("makin map %d,%d",m,n);
        }
    }
    out.data = dataOut;

    out.info.resolution = odomMap.info.resolution;
    out.info.height = sqrt(height*height+width*width);
    out.info.width = out.info.height;
    out.info.map_load_time = odomMap.info.map_load_time;
    out.info.origin.orientation = odomMap.info.origin.orientation;
    geometry_msgs::Point position;

    float tempX = (deca2odomX(0-d2,0-d2)+h2)*out.info.resolution + odomMap.info.origin.position.x;
    float tempY = (deca2odomY(0-d2,0-d2)+w2)*out.info.resolution + odomMap.info.origin.position.y;

    out.info.origin.position.x = odom2decaX(tempX,tempY);
    out.info.origin.position.y = odom2decaY(tempX,tempY);

    out.header.stamp = ros::Time::now();
    out.header.frame_id = "deca";

    return out;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "multiagent_packager");
	ros::NodeHandle nh;	
    //loop every 10 secs
	ros::Rate loop_rate(0.3);
    
    ros::Subscriber position_subscriber = nh.subscribe("odom", 10, positionCallback);
    ros::Subscriber map_subscriber = nh.subscribe("map", 10, mapCallback);
    ros::Subscriber transform_subscriber = nh.subscribe("transform", 10, transformCallback);
	//ros::Publisher robot_publisher = nh.advertise<camp_multiagent::Robot>("robot", 10);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("decamap",10);
    ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PointStamped>("decapos",10);

    //camp_multiagent::Robot robot;
    nav_msgs::OccupancyGrid map;
    geometry_msgs::PointStamped pos;


    while(ros::ok()) {
        pos.header.stamp = ros::Time::now();
        pos.header.frame_id = "deca";
        pos.point = getDecaPosition();

        map = getDecaMap();

	    map_publisher.publish(map);
	    pos_publisher.publish(pos);
        // Check for new messages from subscribed nodes
		ros::spinOnce();
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // ROS_INFO("I SUPNNNED");
	}
}
