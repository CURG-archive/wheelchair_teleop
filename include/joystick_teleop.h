#ifndef WHEELCHAIR_TELEOP_H_
#define WHEELCHAIR_TELEOP_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class WheelchairTeleop
{
    public:
        WheelchairTeleop();

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void publish();

        ros::NodeHandle ph_, nh_;

        int linear_, angular_, deadman_axis_;

        int l_max_up_btn, l_max_down_btn;
        int a_max_up_btn, a_max_down_btn;

        double l_scale_, a_scale_;
        double l_max_, a_max_;

        ros::Publisher vel_pub_;
        ros::Subscriber joy_sub_;

        geometry_msgs::Twist last_published_;
        boost::mutex publish_mutex_;
        bool deadman_pressed_;
        bool zero_twist_published_;
        ros::Timer timer_;

};

#endif /* WHEELCHAIR_TELEOP_H_ */
