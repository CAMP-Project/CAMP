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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
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
geometry_msgs::TransformStamped odom2PointTransform;

// Publisher declaration.
ros::Publisher point_publisher;

// Info for displaying the point in RVIZ.
geometry_msgs::PointStamped test_point;
geometry_msgs::PoseStamped roboPosition;
geometry_msgs::PoseStamped roboInPointPose;

// TF2 Buffer.
tf2_ros::Buffer tfBuffer;


// Callback for robot odometry.
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    ROS_INFO("There is an update happening!");
}

void establishTransformInfo() {
    // tf broadcaster information.
    static tf2_ros::TransformBroadcaster br;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
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
    ros::Rate loop_rate(1);

    // Subscribers.
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 10, &odomCallback);

    // Point publisher.
    point_publisher = nh.advertise<geometry_msgs::PointStamped>("Test_Point", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Functionality.
    while(ros::ok()) {

        // Establishes transformation info for custom point. This method should also sends the transform
        // to the transform tree in RQT.
        test_point.point.x = 2.0;
        test_point.point.y = 2.0;
        test_point.point.z = 2.0;
        test_point.header.stamp = ros::Time::now();
        test_point.header.frame_id = "map";

        // Establishes robot positional information for transform.
        geometry_msgs::PoseStamped roboPosition;
        roboPosition.header.stamp = ros::Time::now();
        roboPosition.header.frame_id = "odom";
        roboPosition.pose.position.x = odom_x;
        roboPosition.pose.position.y = odom_y;
        roboPosition.pose.position.z = 0.0;
        roboPosition.pose.orientation.x = 0.0;
        roboPosition.pose.orientation.y = 0.0;
        roboPosition.pose.orientation.z = 0.0;
        roboPosition.pose.orientation.w = 0.0;

        // Establish transform info from map to point. Also, publish the position of the point to RVIZ.
        establishTransformInfo();
        point_publisher.publish(test_point);


        // Instantiation of a transformStamped to get the transform between odometry and the test point.
        // Also, instantiated a new poseStamped for storing the result of the doTransform method.
        geometry_msgs::TransformStamped odom2PointTransform;
        geometry_msgs::PoseStamped roboInPointPose;
        try {
            tfBuffer.canTransform("point", "odom", ros::Time(0), ros::Duration(3.0));
            odom2PointTransform = tfBuffer.lookupTransform("point", "odom", ros::Time(0));
            tf2::doTransform(roboPosition, roboInPointPose, odom2PointTransform);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Calculate the true distance between the robot and the test point. Compare to the result from tf2.
        float true_x = odom_x - point_origin_x;
        float true_y = odom_y - point_origin_y;
        float tf_x = roboInPointPose.pose.position.x;
        float tf_y = roboInPointPose.pose.position.y;

        ROS_INFO("\n-------True Results-------\nTrue x:   %2.3f   True y:   %2.3f\n-------TF_2 Results-------\nTF2 x:   %2.3f   TF2 y:   %2.3f", true_x, true_y, tf_x, tf_y);
        ros::spinOnce();
    }
    return 0;
}