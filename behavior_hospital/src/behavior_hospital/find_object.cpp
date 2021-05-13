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


#include "behavior_hospital/find_object.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace behavior_hospital
{

FindObject::FindObject(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name,config), ac("move_base",true), buffer_(), listener_(buffer_)
{
    arrived_pub = nh_.advertise<std_msgs::Bool>("/arrived", 1);
}

bool 
FindObject::get_goal()
{
    std::string object;
    geometry_msgs::TransformStamped odom2object_msg;
    if (getInput<std::string>("target").has_value())
    {
        object = getInput<std::string>("target").value();
        ROS_INFO("Objeto que va en la transformada%s",object.c_str());
    }
    try
    {
        bf2object_msg = buffer_.lookupTransform(object, "odom", ros::Time(0));
    }
    catch (std::exception & e)
    {
        ROS_INFO("SI ENTRA MUY MALOOOOOOOOOOOC\n");
        return false;
    }
    goal_.target_pose.header.frame_id = "base_footprint";
    goal_.target_pose.pose.orientation = bf2object_msg.transform.rotation;
    goal_.target_pose.pose.position.x = bf2object_msg.transform.translation.x;
    goal_.target_pose.pose.position.y = bf2object_msg.transform.translation.y;
    goal_.target_pose.pose.position.z = bf2object_msg.transform.translation.z;
    return true;
}

void
FindObject::halt()
{
  ROS_INFO("FindObject halt");
}

BT::NodeStatus
FindObject::tick()
{
    ROS_INFO("FindObject tick");
    if (!(get_goal()))
    {
        return BT::NodeStatus::FAILURE;
    }
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    goal_.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal");
    ac.sendGoal(goal_);

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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