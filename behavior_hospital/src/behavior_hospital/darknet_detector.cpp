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

#include "behavior_hospital/darknet_detector.h"

namespace behavior_hospital
{

DarknetDetector::DarknetDetector(const std::string& name)//, std::string object)
: BT::ActionNodeBase(name, {})//, to_detect_(object)
{
    object_detection_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &DarknetDetector::objectCallback, this);
    finish_detection_ = nh_.advertise<darknet_ros_msgs::BoundingBox>("/detected", 1);
    //to_detect_ = object;
}

void
DarknetDetector::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
{
    int size = box_msg->bounding_boxes.size();

    for (int iter = 0; iter <= size; iter++)
    {
        if (box_msg->bounding_boxes[iter].Class == to_detect_ && box_msg->bounding_boxes[iter].probability > 0.35)
        {
            ROS_INFO("Point %ld,%ld\n", box_msg->bounding_boxes[iter].xmax, box_msg->bounding_boxes[iter].xmin);

            finish_detection_.publish(box_msg->bounding_boxes[iter]);
        }
    }
}

}  // namespace behavior_hospital