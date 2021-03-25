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

*   THIS SOFTWARE IS PROVball_and_goal_bicaED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCball_and_goal_bicaENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "ball_and_goal_bica.h"

namespace bica
{
ball_and_goal_bica::ball_and_goal_bica() : state_(GO_BLUE), myBaseId_("ball_and_goal_bica")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

ball_and_goal_bica::~ball_and_goal_bica()
{
}

void ball_and_goal_bica::activateCode()
{
  	deactivateAllDeps();

	state_ = GO_YELLOW;
	state_ts_ = ros::Time::now();

	Go_yellow_activateDeps();
	Go_yellow_code_once();

}

bool ball_and_goal_bica::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case GO_YELLOW:

	Go_yellow_code_iterative();

	msg.data = "Go_yellow";
	if(Go_yellow_2_Go_blue())
	{

	deactivateAllDeps();

	state_ = GO_BLUE;
	state_ts_ = ros::Time::now();

	Go_blue_activateDeps();
	Go_blue_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TURN:

	Turn_code_iterative();

	msg.data = "Turn";
	if(Turn_2_Go_yellow())
	{

	deactivateAllDeps();

	state_ = GO_YELLOW;
	state_ts_ = ros::Time::now();

	Go_yellow_activateDeps();
	Go_yellow_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GO_BLUE:

	Go_blue_code_iterative();

	msg.data = "Go_blue";
	if(Go_blue_2_Go_ball())
	{

	deactivateAllDeps();

	state_ = GO_BALL;
	state_ts_ = ros::Time::now();

	Go_ball_activateDeps();
	Go_ball_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GO_BALL:

	Go_ball_code_iterative();

	msg.data = "Go_ball";
	if(Go_ball_2_Turn())
	{

	deactivateAllDeps();

	state_ = TURN;
	state_ts_ = ros::Time::now();

	Turn_activateDeps();
	Turn_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
ball_and_goal_bica::deactivateAllDeps()
{
	removeDependency("go_blue");
	removeDependency("go_ball");
	removeDependency("go_yellow");
	removeDependency("turn");
};

void
ball_and_goal_bica::Go_yellow_activateDeps()
{
	addDependency("go_yellow");
}

void
ball_and_goal_bica::Turn_activateDeps()
{
	addDependency("turn");
}

void
ball_and_goal_bica::Go_blue_activateDeps()
{
	addDependency("go_blue");
}

void
ball_and_goal_bica::Go_ball_activateDeps()
{
	addDependency("go_ball");
}



} /* namespace bica */
