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

#ifndef BEHAVIOR_HOSPITAL_ISROOM_H
#define BEHAVIOR_HOSPITAL_ISROOM_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <pcl_ros/transforms.h>

#include "darknet_ros_msgs/BoundingBoxes.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Bool.h"

#define CUADRADO 1

namespace behavior_hospital
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class IsRoom : public BT::ActionNodeBase
{
    public:
    explicit IsRoom(const std::string& name, const BT::NodeConfiguration& config);

    bool get_rooms();
    bool check_room();
    void set_rooms(std::string arg);

    void halt();
    BT::NodeStatus tick();
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("target")};
    }
    
    private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;

    geometry_msgs::Point position_;

    ros::Publisher arrived_pub_;

    tf2_ros::Buffer buffer_;
    tf2_ros::StaticTransformBroadcaster tfBroadcaster_;
    tf2_ros::TransformListener listener_;



    //geometry_msgs::TransformStamped odom2object_msg_;
};

}  // namespace behavior_hospital

#endif  // BEHAVIOR_HOSPITAL_ISROOM_H