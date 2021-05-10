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

#ifndef BEHAVIOR_HOSPITAL_DARKNETDETECTION_H
#define BEHAVIOR_HOSPITAL_DARKNETDETECTION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <pcl_ros/transforms.h>

#include "darknet_ros_msgs/BoundingBoxes.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

namespace behavior_hospital
{

class DarknetDetector : public BT::ActionNodeBase
{
  public:
    explicit DarknetDetector(const std::string& name);//, std::string object);

    void halt();
    BT::NodeStatus tick();

    void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);

  private:
    ros::NodeHandle nh_;

    ros::Subscriber object_detection_;
    ros::Publisher finish_detection_;

    std::string   to_detect_;
    bool done_;
    };

}  // namespace behavior_hospital

#endif  // BEHAVIOR_HOSPITAL_DARKNETDETECTION_H
