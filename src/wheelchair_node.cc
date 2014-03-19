#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include "../include/wheelchair_node.h"

WheelchairNode::WheelchairNode():
    ph_("~"),
    decay_(0.001),
    offset_(-0.1),
    angle_(0)
{
    ph_.param("decay_constant", decay_, decay_);
    ph_.param("offset", offset_, offset_);
    joint_pub_ = ph_.advertise<sensor_msgs::JointState>("kinect_joint_state", 1, true);
    tilt_sub_ = nh_.subscribe<std_msgs::Float64>("cur_tilt_angle", 10, &WheelchairNode::tiltCallback, this);
}

void WheelchairNode::tiltCallback(const std_msgs::Float64::ConstPtr& tilt) {
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "camera_link";

    js.name.clear();
    js.name.push_back("kinect_joint");

    js.position.clear();

    double new_angle = -tilt->data * 3.1415926535 / 180;

    // offset because the kinect accelerometer isn't very good
    new_angle += offset_;

    // apply low-pass filter
    angle_ = decay_ * new_angle + (1 - decay_) * angle_;

    js.position.push_back(angle_);
    
    js.velocity.clear();
    js.velocity.push_back(0);

    js.effort.clear();
    js.effort.push_back(0);

    joint_pub_.publish(js);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheelchair_node");
    WheelchairNode wheelchair_node;

    ros::spin();
}

