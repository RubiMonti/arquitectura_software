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

*   THIS SOFTWARE IS PROVBehavior_hospitalED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCBehavior_hospitalENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "Behavior_hospital.h"

namespace bica
{
Behavior_hospital::Behavior_hospital() : state_(GO_GOAL), myBaseId_("Behavior_hospital")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

Behavior_hospital::~Behavior_hospital()
{
}

void Behavior_hospital::activateCode()
{
  	deactivateAllDeps();

	state_ = GO_GOAL;
	state_ts_ = ros::Time::now();

	Go_goal_activateDeps();
	Go_goal_code_once();

}

bool Behavior_hospital::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case GO_GOAL:

	Go_goal_code_iterative();

	msg.data = "Go_goal";
	if(Go_goal_2_Find_object())
	{

	deactivateAllDeps();

	state_ = FIND_OBJECT;
	state_ts_ = ros::Time::now();

	Find_object_activateDeps();
	Find_object_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FIND_OBJECT:

	Find_object_code_iterative();

	msg.data = "Find_object";
	if(Find_object_2_Go_goal())
	{

	deactivateAllDeps();

	state_ = GO_GOAL;
	state_ts_ = ros::Time::now();

	Go_goal_activateDeps();
	Go_goal_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
Behavior_hospital::deactivateAllDeps()
{
	removeDependency("Go");
	removeDependency("Find");
};

void
Behavior_hospital::Go_goal_activateDeps()
{
	addDependency("Go");
}

void
Behavior_hospital::Find_object_activateDeps()
{
	addDependency("Find");
}



} /* namespace bica */
