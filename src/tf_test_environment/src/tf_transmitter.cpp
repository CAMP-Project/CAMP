// This is a comment!

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

//--------------------------------------------
// TF2 inclusions.
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
//--------------------------------------------

#include <list>
#include <math.h>

std::string turtle_name;

// This is a method grabbed from the tutorial. Callback for sending the transform 
// of the turtlesim pose.
void poseCallback(const turtlesim::PoseConstPtr& msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Establish the transform information.
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = turtle_name;
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = mag->y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transformStemped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_transmitter");
	ros::NodeHandle private_node("~");
    if (!private_node.hasParam("turtle")) {
        if (argc != 2) {
            ROS_ERROR("need turtle name as argument");
        }
        turtle_name = argv[1];
    } else {
        private_node.getParam("turtle", turtle_name);
    }	

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback)

	ros::Rate loop_rate(10);



    return 0;
}