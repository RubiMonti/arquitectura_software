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

class DarknetDetection
{
public:
    explicit DarknetDetection(char *object)
    {
        object_detection_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &DarknetDetection::objectCallback, this);
        finish_detection_ = nh_.advertise<darknet_ros_msgs::BoundingBox>("/detected", 1);
        to_detect_ = object;
    }

    void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
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

private:
    ros::NodeHandle nh_;

    ros::Subscriber object_detection_;
    ros::Publisher finish_detection_;

    float coor2dx_;
    float coor2dy_;
    float coor3dx_;
    float coor3dy_;
    float coor3dz_;
    char *to_detect_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet");

    DarknetDetection dd = DarknetDetection(argv[2]);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
