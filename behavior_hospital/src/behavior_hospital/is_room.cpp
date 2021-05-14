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


#include "behavior_hospital/is_room.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace behavior_hospital
{

IsRoom::IsRoom(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config), buffer_(), listener_(buffer_)
{
    arrived_pub = nh_.advertise<std_msgs::Bool>("/arrived", 1);
}

void IsRoom::set_rooms(std::string arg)
{
    if (arg == "room1")
    {
        position_.x = -6.13;
        position_.y = 8.2;
    }
    else if (arg == "room2")
    {
        position_.x = -5.97;
        position_.y = -8.21;
    }
    else if (arg== "hall")
    {
        position_.x = 0.0;
        position_.y = 0.0;
    }
    else if (arg == "office1")
    {
        position_.x = -5.37;
        position_.y = 1.13;
    }
    else if (arg == "office2")
    {
        position_.x = -5.22;
        position_.y = -1.37;
    }
    else if (arg == "storage1")
    {
        position_.x = 10.93;
        position_.y = 8.14;
    }
    else if (arg == "storage2")
    {
        position_.x = 10.77;
        position_.y = -8.21;
    }
    else
    {
        ROS_INFO("No input detected.\nPossible inputs: Office1, Office2, Hall, Room1, Room2, Storage1, Storage2.\n");
    }
}

bool
check_room()
{
    geometry_msgs::TransformStamped map2bf_msg;
    try
    {
        map2bf_msg = buffer_.lookupTransform("map", "base_footprint", ros::Time(0));
    }
    catch (std::exception & e)
    {
        return false;
    }
    if ((map2bf_msg.transform.translation.x < position_.x + CUADRADO && map2bf_msg.transform.translation.x > position_.x - CUADRADO) &&
        (map2bf_msg.transform.translation.y < position_.y + CUADRADO && map2bf_msg.transform.translation.y > position_.y - CUADRADO))
    {
        return true;
    }
    return false;
}

void
IsRoom::halt()
{
  ROS_INFO("IsRoom halt");
}

BT::NodeStatus
IsRoom::tick()
{
    std_msgs::Bool msg;

    ROS_INFO("IsRoom tick");
    std::string room;
    if (getInput<std::string>("target").has_value())
    {
        room = getInput<std::string>("target").value();
        ROS_INFO("%s", room.c_str());
    }
    set_rooms(room);
    if (check_room())
    {
        ROS_INFO("Already in room");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_INFO("Not in the room");
        return BT::NodeStatus::FAILURE;
    }
}
}  // namespace behavior_hospital
