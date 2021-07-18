/* 
This function should be run in additioon to other nodes to calibrate the odom/deca conversion.
Started 4/15/2021 by Tyler Pigott
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <list>
#include <math.h>
#define PI 3.14159265

using namespace std;

// Parameters for later
const int min_point_count = 3;
const int max_point_count = 20;
const int max_point_modification = 15;
const float new_point_distance = 0.1;
const float bad_theta_threshold = 0.2;
const float bad_data_threshold = 5;
const float theta_found_threshold = 0.001;
const float d_theta = 0.0001;
float theta_0 = 0;
int max_point_modifier = 0;

list<float> odom_x, odom_y, deca_x, deca_y;
float odom_in_x = -999, odom_in_y, deca_in_x = -999, deca_in_y;

// Callback for grabbing odometry information.
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_in_x = msg->pose.pose.position.x;
    odom_in_y = msg->pose.pose.position.y;
}

// Callback for grabbing Decawave Tag information.
void tagCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    deca_in_x = msg->linear.x;
    deca_in_y = msg->linear.y;
}

// Mass coordinate vector commands
void pushCoords() {
    odom_x.push_back(odom_in_x);
    odom_y.push_back(odom_in_y);
    deca_x.push_back(deca_in_x);
    deca_y.push_back(deca_in_y);
    while (odom_x.size() > max_point_count - max_point_modifier) {
        odom_x.pop_front();
        odom_y.pop_front();
        deca_x.pop_front();
        deca_y.pop_front();        
    }
    max_point_modifier = 0;
}
void clearCoords() {
    odom_x.clear();
    odom_y.clear();
    deca_x.clear();
    deca_y.clear();
}

// Method for computing new offsets to convert between the odometry frame and the decawave frame.
geometry_msgs::TransformStamped getOffsets(ros::Publisher transform_publisher, geometry_msgs::TransformStamped lastTransform) {
    geometry_msgs::Vector3 tr;
    std::list<float>::iterator ox, oy, dx, dy;
    float tx,ty,theta,last_theta,run;
    float a,b,a2,b2,tx2,ty2,n,f,f2,f_prime,e;

    theta = theta_0;
    last_theta = theta_0;
    run = 1;
    int attempt = 1;

    while(run == 1){
        run = 0;
        a = cos(theta);
        b = sin(theta);
        a2 = cos(theta + d_theta);
        b2 = sin(theta + d_theta);
        tx = 0;
        ty = 0;
        tx2 = 0;
        ty2 = 0;
        n = odom_x.size();

        
        oy = odom_y.begin();
        dx = deca_x.begin();
        dy = deca_y.begin();
        for(ox = odom_x.begin(); ox != odom_x.end(); ++ox){
            tx = tx + (1/n)*(*dx-a**ox+b**oy);
            ty = ty + (1/n)*(*dy-a**oy-b**ox);
            tx2 = tx2 + (1/n)*(*dx-a2**ox+b2**oy);
            ty2 = ty2 + (1/n)*(*dy-a2**oy-b2**ox);
            ++oy;
            ++dx;
            ++dy;
        }
        f = 0;
        f2 = 0;
        oy = odom_y.begin();
        dx = deca_x.begin();
        dy = deca_y.begin();
        for(ox = odom_x.begin(); ox != odom_x.end(); ++ox){
            f = f + (ty-*dy)*(a**ox-b**oy) - (tx-*dx)*(b**ox+a**oy);
            f2 = f2 + (ty2-*dy)*(a2**ox-b2**oy) - (tx2-*dx)*(b2**ox+a2**oy);
            ++oy;
            ++dx;
            ++dy;
        }
        f_prime = (f2 - f)/d_theta;
        last_theta = theta;
        theta = theta - f/f_prime;
        if (f_prime == 0) f_prime = 0.00000001;
        if (theta > PI || theta < -PI) theta = theta - floor(theta/(PI))*PI;
        if (theta == 0) theta = 0.00000001;
        if (abs((theta - last_theta)/theta) > theta_found_threshold) 
            run = 1; 
        else {
            e = 0;
            oy = odom_y.begin();
            dx = deca_x.begin();
            dy = deca_y.begin();
            for(ox = odom_x.begin(); ox != odom_x.end(); ++ox){
                e = e + pow((a**ox-b**oy+tx-*dx),2)+pow((b**ox+a**oy+ty-*dy),2);
                ++oy;
                ++dx;
                ++dy;
            }
            e = e/n;
            ROS_INFO("-- Attempt: %d --",attempt);
            ROS_INFO("Error: %f",e);
            ROS_INFO("Theta: %f",theta);
            if(e > bad_theta_threshold) {\
                if (theta != 0) {
                    theta = theta - theta/abs(theta)*PI;    
                } else theta = PI;
                run = 1;
                attempt++;
                if(attempt > 3) {
                    max_point_modifier = max_point_modification;
                    return lastTransform;
                }

            }
            //else if(e > bad_data_threshold) do something that fixes it i guess? throw everything out?
        }

    }

    ROS_INFO("I am reaching the publish point!");
    theta_0 = theta;

    tr.x = tx;
    tr.y = ty;
    tr.z = theta;
    ROS_INFO("pos offsets: (%f/%f)",tx,ty);
    ROS_INFO("Theta: %f",theta);
	transform_publisher.publish(tr);
    //tf2 stuff
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "deca";
    transformStamped.transform.translation.x = tx;
    transformStamped.transform.translation.y = ty;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ROS_INFO("I am reaching the broadcast point!");

    
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);

    return transformStamped;
}

void broadcastTransform(float tx, float ty, float theta) {

    // Instantiate a broadcast node and a stamped transform.
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Fill the stamped transform with the appropriate translational information.
    transformStamped.header.stamp = ros::Time::now(); // Timestamp.
    transformStamped.header.frame_id = "odom";        // Parent frame id.
    transformStamped.child_frame_id = "deca";         // Child framd id.
    transformStamped.transform.translation.x = tx;    // Translation in the x direction.
    transformStamped.transform.translation.y = ty;    // Translation in the y direction.
    transformStamped.transform.translation.z = 0.0;
    
    // Fill the stamped transform with the appropriate rotational information.
    tf2::Quaternion q; 
    q.setRPY(0, 0, theta); // Specify the quaternion.
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ROS_INFO("I am reaching the broadcast point!");

    
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle nh;	
	ros::Rate loop_rate(10);
    
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "deca";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber tag_subscriber = nh.subscribe("filtered", 10, tagCallback);
	ros::Publisher transform_publisher = nh.advertise<geometry_msgs::Vector3>("transform", 10);
    
    while(ros::ok()) {

        // Do this if there are no point syet
        if(odom_in_x != -999 && deca_in_x != -999 && odom_x.size() == 0) {
            ROS_INFO("First Push");
            pushCoords();
            ROS_INFO("size: %d",odom_x.size());
        }

        // Do this if its time for another point
        if(sqrt(pow((odom_x.back() - odom_in_x),2) + pow((odom_y.back() - odom_in_y),2))>new_point_distance && odom_x.size() != 0) {
            pushCoords();
            ROS_INFO("size: %d",odom_x.size());

            // Do this if there are enough points to do a conversion
            if(odom_x.size() >= min_point_count){
                ROS_INFO("Publishing (new)...");
                getOffsets(transform_publisher,transformStamped);
            } else {
                broadcastTransform(0.0, 0.0, 0.0);
            }
        } else {
            broadcastTransform(0.0, 0.0, 0.0);
        }
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}