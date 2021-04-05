/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Rubén Montilla ruben.montilla.fdez@gmail.com */

/* Mantainer: Rubén Montilla ruben.montilla.fdez@gmail.com */

#include "ball_and_goal/FindYellowGoal.h"


#include "geometry_msgs/Twist.h"
#include "bica/Component.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ball_and_goal_bica
{

FindYellowGoal::FindYellowGoal() : it_(nh_) , buffer_() , listener_(buffer_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FindYellowGoal::imageCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    sub_laser_ = nh_.subscribe("/scan", 1, &FindYellowGoal::laserCallback, this);

}

void 
FindYellowGoal::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!isActive()){
        return;
    }
    dist_centro_ = msg->ranges[msg->ranges.size()/2];
}

void
FindYellowGoal::publish_detection(float x, float y)
{
    double angle;
    geometry_msgs::TransformStamped odom2bf_msg;
    try{
        odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
    }   catch (std::exception & e)
    {
        return;
    }

    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);

    tf2::Stamped<tf2::Transform> bf2yellow_goal;
    bf2yellow_goal.setOrigin(tf2::Vector3(x, y ,0));
    bf2yellow_goal.setRotation(tf2::Quaternion(0, 0, 0, 1));

    tf2::Transform odom2yellow_goal = odom2bf * bf2yellow_goal;

    geometry_msgs::TransformStamped odom2yellow_goal_msg;
    odom2yellow_goal_msg.header.stamp = ros::Time::now();
    odom2yellow_goal_msg.header.frame_id = "odom";
    odom2yellow_goal_msg.child_frame_id = "yellow_goal";

    odom2yellow_goal_msg.transform = tf2::toMsg(odom2yellow_goal);

    broadcaster_.sendTransform(odom2yellow_goal_msg);

}

void
FindYellowGoal::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
    if(!isActive()){
        return;
    }

    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // imagen que me acaba de llegar

    cv::Mat hsv;
    cv:cvtColor(cv_ptr->image , hsv, CV_RGB2HSV);

    int height = hsv.rows;
    int width = hsv.cols;
    int step = hsv.step;
    int channels = 3;  // RGB

    x_ = 0;
    y_ = 0;
    counter_ = 0;
    for ( int i=0; i < height; i++ )
    {
        for ( int j=0; j < width; j++ )
        {
            int posdata = i * step + j * channels;
            if ((hsv.data[posdata] >= 90) && (hsv.data[posdata] <= 92)
             && (hsv.data[posdata+1]  >= 0) && (hsv.data[posdata+1] <= 255)
             && (hsv.data[posdata+2]  >=0) && (hsv.data[posdata+2] <= 255))
            {
                x_ += j;
                y_ += i;
                counter_++;
            }
        }
    }
}

void
FindYellowGoal::step()
{
    if(!isActive()){
        return;
    }
    geometry_msgs::Twist msg2;

    int pos_x, pos_y;
    bool found = true;
    double angle;

    geometry_msgs::TransformStamped bf2yellow_goal_2_msg;
    try {
        bf2yellow_goal_2_msg = buffer_.lookupTransform( "base_footprint", "yellow_goal", ros::Time(0));
    } catch (std::exception & e)
    {
        found = false;
    }

    if (found)
    {
        angle = atan2(bf2yellow_goal_2_msg.transform.translation.y, bf2yellow_goal_2_msg.transform.translation.x);

        if (angle < -1)
        {
            msg2.angular.z = -0.5;
        }
        else if (angle > 1)
        {
            msg2.angular.z = 0.5;
        }
        else if(angle > 0.1)
        {
            msg2.angular.z = 0.2;
        }
        else if(angle < -0.1)
        {
            msg2.angular.z = -0.2;
        }
        else
        {
            msg2.linear.x = 0.2;
            if (dist_centro_ < 0.4)
            {
                msg2.linear.x = 0.0;
                msg2.angular.z = 0.0;
            }
        }
    }

    else if (!found && counter_ > 500)
    {
        pos_x = x_ / counter_;
        pos_y = y_ / counter_;
        msg2.angular.z = 0.2;
        if (dist_centro_ < 0.4)
        {
            msg2.linear.x = 0.0;
            msg2.angular.z = 0.0;

            publish_detection(0.4,0);
        }
        else
        {
             if (pos_x >= 300 && pos_x <= 340)
            {
                msg2.linear.x = 0.2;
                msg2.angular.z = 0.0;
            }
            else if(pos_x >= 270 && pos_x <= 300)
            {
                msg2.linear.x = 0.2;
                msg2.angular.z = 0.1;
            }
            else if(pos_x >= 340 && pos_x <= 370)
            {
                msg2.linear.x = 0.25;
                msg2.angular.z = -0.1;
            }
        }
    }
    else
    {
        msg2.angular.z = 0.5;
    }
    vel_pub_.publish(msg2);
}
} // namespace ball_and_goal_bica