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

#ifndef BALL_AND_GOAL_BALLANDGOALBICA_H
#define BALL_AND_GOAL_BALLANDGOALBICA_H

#include "./ball_and_goal_bica.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace ball_and_goal_bica
{
class BallAndGoalBica : public bica::ball_and_goal_bica
{
public:
    BallAndGoalBica();

    bool Turn_2_Go_yellow();
    bool Go_ball_2_Turn();
    bool Go_blue_2_Go_ball();
    bool Go_yellow_2_Go_blue();

    void Go_yellow_code_once();
    void Go_blue_code_once();
    void Go_ball_code_once();
    void Turn_code_once();

private:
    void CAMBIAR_callback(const std_msgs::Bool::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::Subscriber obstacle_sub_;
    bool is_obstacle_;
};

}  // namespace ball_and_goal_bica

#endif  // BALL_AND_GOAL_BALLANDGOALBICA_H
