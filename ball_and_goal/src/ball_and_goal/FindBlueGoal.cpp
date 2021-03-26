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

#include "ball_and_goal/FindBlueGoal.h"


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

FindBlueGoal::FindBlueGoal() : it_(nh_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FindBlueGoal::imageCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
FindBlueGoal::imageCb(const sensor_msgs::Image::ConstPtr& msg)
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
            if ((hsv.data[posdata] >= 0) && (hsv.data[posdata] <= 153)
             && (hsv.data[posdata+1]  >= 176) && (hsv.data[posdata+1] <= 223)
             && (hsv.data[posdata+2]  >=70) && (hsv.data[posdata+2] <= 92))
            {
                x_ += j;
                y_ += i;
                counter_++;
            }
        }
    }
}

void
FindBlueGoal::step()
{
    if(!isActive()){
        return;
    }
    geometry_msgs::Twist msg2;

    int pos_x, pos_y;
    if (counter_ > 500)
    {
        ROS_INFO("\nGoal at %d %d\n", x_ / counter_ , y_ / counter_);
        pos_x = x_ / counter_;
        pos_y = y_ / counter_;
        msg2.angular.z = 0.2;
        if (pos_x >= 200 && pos_x <= 300)
        {
            msg2.linear.x = 0.2;
            msg2.angular.z = 0.0;
        }
    }
    else
    {
        ROS_INFO("\nNO GOAL FOUND\n");
        msg2.angular.z = 0.5;
    }
    vel_pub_.publish(msg2);
}

}  // namespace ball_and_goal_bica
