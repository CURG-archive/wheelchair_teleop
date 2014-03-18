/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include "../include/joystick_teleop.h"

WheelchairTeleop::WheelchairTeleop():
    ph_("~"),
    linear_(1),
    angular_(0),
    l_max_up_btn(4),
    l_max_down_btn(6),
    a_max_up_btn(5),
    a_max_down_btn(7),
    deadman_axis_(4),
    l_scale_(0.3),
    a_scale_(0.9),
    l_max_(0.3),
    a_max_(0.9)
{
    ph_.param("axis_linear", linear_, linear_);
    ph_.param("axis_angular", angular_, angular_);
    ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
    ph_.param("linear_max_up_btn", l_max_up_btn, l_max_up_btn);
    ph_.param("linear_max_down_btn", l_max_down_btn, l_max_down_btn);
    ph_.param("angular_max_up_btn", a_max_up_btn, a_max_up_btn);
    ph_.param("angular_max_down_btn", a_max_down_btn, a_max_down_btn);
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);

    // default to speed limits at maximum
    l_max_ = l_scale_ / 2;
    a_max_ = a_scale_ / 2;

    deadman_pressed_ = false;
    zero_twist_published_ = false;

    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &WheelchairTeleop::joyCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&WheelchairTeleop::publish, this));
}

void WheelchairTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
    geometry_msgs::Twist vel;

    bool l_up = joy->buttons[l_max_up_btn];
    bool l_down = joy->buttons[l_max_down_btn];
    bool a_up = joy->buttons[a_max_up_btn];
    bool a_down = joy->buttons[a_max_down_btn];

    if (l_up || l_down || a_up || a_down) {
        if (l_up) {
            l_max_ += 0.1;
        } else if (l_down) {
            l_max_ -= 0.1;
            if (l_max_ < 0) {
                l_max_ = 0;
            }
            if (l_max_ > l_scale_) {
                l_max_ = l_scale_;
            }
        }

        if (a_up) {
            a_max_ += 0.1;
        } else if (a_down) {
            a_max_ -= 0.1;
            if (a_max_ < 0) {
                a_max_ = 0;
            }
            if (a_max_ > a_scale_) {
                a_max_ = a_scale_;
            }
        }

        ROS_INFO("CONSTRAINTS set to (%.02f, %.02f)", l_max_, a_max_);
    }

    bool constrained = false;
    // turn
    vel.angular.z = a_scale_ * joy->axes[angular_];
    if (vel.angular.z < -a_max_) {
        vel.angular.z = -a_max_;
        constrained = true;
    } else if (vel.angular.z > a_max_) {
        vel.angular.z = a_max_;
        constrained = true;
    }

    // forward
    vel.linear.x = l_scale_ * joy->axes[linear_];
    if (vel.linear.x < -l_max_) {
        vel.linear.x = -l_max_;
        constrained = true;
    } else if (vel.linear.x > l_max_) {
        vel.linear.x = l_max_;
        constrained = true;
    }

    if (constrained) {
        ROS_WARN("cmd_vel constrained to (%.02f, %.02f)", vel.linear.x, vel.angular.z);
    }

    last_published_ = vel;

    deadman_pressed_ = joy->buttons[deadman_axis_];
}

void WheelchairTeleop::publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    if (deadman_pressed_)
    {
        vel_pub_.publish(last_published_);
        zero_twist_published_ = false;
    }
    else if(!deadman_pressed_ && !zero_twist_published_)
    {
        vel_pub_.publish(*new geometry_msgs::Twist());
        zero_twist_published_ = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheelchair_teleop");
    WheelchairTeleop wheelchair_teleop;

    ros::spin();
}

