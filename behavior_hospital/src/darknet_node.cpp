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
#include "std_msgs/Bool.h"




class DarknetDetection
{
public:
    explicit DarknetDetection(std::string object)
    {
        object_detection_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &DarknetDetection::objectCallback, this);
        finish_detection_ = nh_.advertise<darknet_ros_msgs::BoundingBox>("/detected", 1);
        arrived_sub_ = nh_.subscribe("arrived", 1, &DarknetDetection::messageCallback, this);
        to_detect_ = object;
    }

    void messageCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        //ROS_INFO("Message: [%i]", msg->data);
        arrived_ = msg->data;
    }
    void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
    {
        int size = box_msg->bounding_boxes.size();
        ROS_INFO("PATATAAAAAAA");
        ROS_INFO("%i",arrived_);


        if(arrived_)
        {
            for (int iter = 0; iter <= size; iter++)
            {
                if (box_msg->bounding_boxes[iter].Class == to_detect_ && box_msg->bounding_boxes[iter].probability >= 0.35)
                {
                    ROS_INFO("Point xmin: %ld, xmax: %ld\n", box_msg->bounding_boxes[iter].xmin, box_msg->bounding_boxes[iter].xmax);
                    ROS_INFO("Point ymin: %ld, ymax: %ld\n", box_msg->bounding_boxes[iter].ymin, box_msg->bounding_boxes[iter].ymax);

                    finish_detection_.publish(box_msg->bounding_boxes[iter]);
                }
            }
        }
        else{
            ROS_INFO("LLEGANDO A HABITACION");
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber object_detection_;
    ros::Publisher finish_detection_;

    float coor2dx_;
    float coor2dy_;
    float coor3dx_;
    float coor3dy_;
    float coor3dz_;
    std::string to_detect_;
    ros::Subscriber arrived_sub_;
    bool arrived_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet");

    DarknetDetection dd = DarknetDetection(argv[2]);

    ros::spin();
  
    return 0;
}
