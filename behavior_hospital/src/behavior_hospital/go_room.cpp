// Copyright 2021 ROScon de Reyes
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "behavior_hospital/go_room.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

namespace behavior_hospital
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

GoRoom::GoRoom(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    // Aqui tendriamos que meter la habitacion
    room_ = "hall";
    first_ = true;
}

void 
GoRoom::set_goal(move_base_msgs::MoveBaseGoal& goal, std::string arg)
{
    if (arg == "room1")
    {
        ROS_INFO("Going to room1\n");
        goal.target_pose.pose.position.x = -6.13;
        goal.target_pose.pose.position.y = 8.2;
    }
    else if (arg == "room2")
    {
        ROS_INFO("Going to room2\n");
        goal.target_pose.pose.position.x = -5.97;
        goal.target_pose.pose.position.y = -8.21;
    }
    else if (arg== "hall")
    {
        ROS_INFO("Going to hall\n");
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
    }
    else if (arg == "office1")
    {
        ROS_INFO("Going to offcie1\n");
        goal.target_pose.pose.position.x = -5.37;
        goal.target_pose.pose.position.y = 1.13;
    }
    else if (arg == "office2")
    {
        ROS_INFO("Going to office2\n");
        goal.target_pose.pose.position.x = -5.22;
        goal.target_pose.pose.position.y = -1.37;
    }
    else if (arg == "storage1")
    {
        ROS_INFO("Going to storage1\n");
        goal.target_pose.pose.position.x = 12.5;
        goal.target_pose.pose.position.y = 7.89;
    }
    else if (arg == "storage2")
    {
        ROS_INFO("Going to storage2\n");
        goal.target_pose.pose.position.x = 12.47;
        goal.target_pose.pose.position.y = -7.72;
    }
    else
    {
        ROS_INFO("No input detected.\nPossible inputs: Office1, Office2, Hall, Room1, Room2, Storage1, Storage2.\n");
    }
    goal.target_pose.pose.orientation.w = 0.0013;
}

/*
static void
GoRoom::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}
*/

void
GoRoom::halt()
{
  ROS_INFO("GoRoom halt");
}

BT::NodeStatus
GoRoom::tick()
{
    ROS_INFO("GoRoom tick");
    MoveBaseClient ac("move_base",true);
    if (first_)
    {
        
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        goal_.target_pose.header.frame_id = "map";
        set_goal(goal_, room_);
        goal_.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("Sending goal");
        ac.sendGoal(goal_);// , doneCb);
        first_ = false;
    }
    // ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE || 
        ac.getState() == actionlib::SimpleClientGoalState::PENDING)
    {
        ROS_INFO("Mission is being accomplished");
        return BT::NodeStatus::RUNNING;
    }
    else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, mission accomplished");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_INFO("[Error] mission could not be accomplished");
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace behavior_hospital