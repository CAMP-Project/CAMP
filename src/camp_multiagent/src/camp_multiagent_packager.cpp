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

geometry_msgs::PointStamped odomPosition;
nav_msgs::OccupancyGrid odomMap;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odomPosition.point = msg->pose.pose.position;
    odomPosition.header = msg->header;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    odomMap.data = msg->data;
    odomMap.header = msg->header;
    odomMap.info = msg->info;
}

geometry_msgs::Point getDecaPosition(geometry_msgs::PointStamped in){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::PointStamped out;
    try {
        out = tfBuffer.transform<geometry_msgs::PointStamped>(in,"deca", ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
    return out.point;
}

std::vector<signed char> getDecaMap(nav_msgs::OccupancyGrid map){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped d2o;
    geometry_msgs::PoseStamped originIn, originOut;
    std::vector<signed char> dataIn, dataOut;
    int width, height, newDimensions;
    int w2,h2,d2;
    int mIn, nIn;
    float resolution, theta, c, s;

    //Get a bunch of map data
    originIn.header = map.header;
    originIn.pose = map.info.origin;
    dataIn = map.data;
    width = map.info.width;
    height = map.info.height;
    resolution = map.info.resolution;

    //Transform the map origin to decawave (do i even want this?)
    // try {
    //     originOut = tfBuffer.transform<geometry_msgs::PoseStamped>(originIn,"deca", ros::Duration(0.1));
    // } catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     continue;
    // }

    //Decide how big the new map is.
    //the old map can't be wider than it's diagonal at any rotation, so we will use that as our new width and height.
    newDimensions = sqrt(height*height+width*width);

    // go through each element in the new array and fill it in with something
    //det the angle that changes decawave points to odometry
    d2o = tfBuffer.lookupTransform("odom","deca",ros::Time(0),ros::Duration(0.1));
    //assuming rotation is only about the z axis
    theta = acos(d2o.transform.rotation.w)*2;
    //other useful params
    w2 = width/2;
    h2 = height/2;
    d2 = newDimensions/2;
    c = cos(theta);
    s = sin(theta);
    //with maps centers alligned, take every element of our new decawave map and find what value on the odom map correlates.
    for(int m = 0; m < newDimensions; m++){
        for(int n = 0; n < newDimensions; n++){
            mIn = h2 + c*(m-d2)-s*(n-d2);
            nIn = w2 + s*(m-d2)+c*(n-d2);
            if (mIn >= 0 && mIn < height && nIn >= 0 && nIn < width) {
                dataOut[m*newDimensions+n] = dataIn.at((mIn)*height+(nIn));
            } else {
                dataOut[m*newDimensions+n] = -1;
            }
        }
    }

    return dataOut;
}

nav_msgs::MapMetaData getDecaInfo(nav_msgs::OccupancyGrid map){
    return map.info;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "multiagent_packager");
	ros::NodeHandle nh;	
    //loop every 10 secs
	ros::Rate loop_rate(0.1);
    
    ros::Subscriber position_subscriber = nh.subscribe("odom", 10, positionCallback);
    ros::Subscriber map_subscriber = nh.subscribe("map", 10, mapCallback);
	ros::Publisher robot_publisher = nh.advertise<camp_multiagent::Robot>("robot", 10);

    camp_multiagent::Robot robot;

    while(ros::ok()) {
        robot.header.stamp = ros::Time::now();
        robot.header.frame_id = "deca";

        robot.position = getDecaPosition(odomPosition);
        robot.data = getDecaMap(odomMap);
        robot.info = getDecaInfo(odomMap);

	    robot_publisher.publish(robot);
        // Sleep according to the loop rate above
		loop_rate.sleep();
        // Check for new messages from subscribed nodes
		ros::spinOnce();
	}
}
