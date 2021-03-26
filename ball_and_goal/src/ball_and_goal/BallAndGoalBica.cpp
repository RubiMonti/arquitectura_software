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

# include "ros/ros.h"
# include "ball_and_goal/BallAndGoalBica.h"

# define MAX_TIME 30.0

namespace ball_and_goal_bica 
{

BallAndGoalBica::BallAndGoalBica() 
{
}

bool
BallAndGoalBica::Go_yellow_2_Go_blue()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Go_blue_2_Go_ball()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Go_ball_2_Turn()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Turn_2_Go_yellow()
{
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
}

}  // ball_and_goal