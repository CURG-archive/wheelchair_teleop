#ifndef WHEELCHAIR_NODE_H_
#define WHEELCHAIR_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class WheelchairNode
{
    public:
        WheelchairNode();

    private:
        void tiltCallback(const std_msgs::Float64::ConstPtr& tilt);

        double angle_, decay_, offset_;
        ros::NodeHandle ph_, nh_;
        ros::Subscriber tilt_sub_;
        ros::Publisher joint_pub_;
};

#endif /* WHEELCHAIR_NODE_H_ */
