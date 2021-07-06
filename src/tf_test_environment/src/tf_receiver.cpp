//--------------------------------------------
// Message Inclusions.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
//--------------------------------------------
// TF2 inclusions.
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <turtlesim/Pose.h>
//--------------------------------------------
// Normal C++ Inclusions.
#include <list>
#include <math.h>
//--------------------------------------------

/*
------------------------------------------------------------------------------------------------------------
To test the functionality of the tf2 package. This code will perform the following actions:

1. Create a pointStamped object with a pose/position, along with the corresponding transform.
2. Subscribe to the Turtlebot odometry to use the robot's real position (will be useful for real tests.)
3. Use the information of the pointStamp to find the robot's position relative to the pointStamp. 
4. The resulting information will be compared to the robot's theoretically calculated position relative
   to the pointStamp.
------------------------------------------------------------------------------------------------------------
*/

// Odometry variables.
float odom_x;
float odom_y;

// Origin of test point.
float point_origin_x = 2.0;
float point_origin_y = 2.0;

// Initialize a transformStamped object.
geometry_msgs::TransformStamped transformStamped;

// Publisher declaration.
ros::Publisher point_publisher;

// Info for displaying the point in RVIZ.
geometry_msgs::PointStamped test_point;

// Callback for robot odometry.
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
}

void establishTransformInfo() {
    // tf broadcaster information.
    static tf2_ros::TransformBroadcaster br;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "test_point";
    transformStamped.child_frame_id = "point";
    transformStamped.transform.translation.x = point_origin_x;
    transformStamped.transform.translation.y = point_origin_y;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    // ROS initialization stuff.
    ros::init(argc, argv, "tf_transmitter");
	ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Subscribers.
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, &odomCallback);

    // Point publisher.
    point_publisher = nh.advertise<geometry_msgs::PointStamped>("Test_Point", 10);

    // Functionality.
    while(ros::ok()) {

        // Establishes transformation info for custom point. This method should also sends the transform
        // to the transform tree in RQT.
        test_point.point.x = 2.0;
        test_point.point.y = 2.0;
        test_point.point.z = 2.0;
        test_point.header.stamp = ros::Time::now();
        test_point.header.frame_id = "test_point";

        establishTransformInfo();
        point_publisher.publish(test_point);


        // NEED A BLOCK FOR TRANSFORM CALCULATION WITH TF***



        // NEED TO CALCULATE TRUE POSITION WITH MATH***


    }

    return 0;
}