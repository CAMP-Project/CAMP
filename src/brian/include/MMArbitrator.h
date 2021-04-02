#ifndef MM_ARBITRATOR_H
#define MM_ARBITRATOR_H

/*
 * This header files holds the class for the Multi Master Arbitrator
 * The Multi Master Arbitrator is responsible for discovering additional
 * robots on the ROS network and sync up the various nodes with the current
 * robot.
 */ 

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <vector>

class MMArbitrator
{
public:
    MMArbitrator();

    void sync(ros::NodeHandle);

    ~MMArbitrator();
private:
    std::vector<ros::Subscriber *> _subs;
    std::vector<ros::Publisher *> _pubs;

    ros::Subscriber _mm_entry_point;
};

#endif // MM_ARBITRATOR_H