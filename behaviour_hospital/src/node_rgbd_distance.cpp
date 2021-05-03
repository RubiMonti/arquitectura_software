// Copyright 2019 Intelligent Robotics Lab
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
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <pcl_ros/transforms.h>

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RGBDFilter
{
public:
  RGBDFilter()
  {
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    double x,y,z;
    sensor_msgs::PointCloud2 cloud;

    try
    {
      pcl_ros::transformPointCloud("camera_link", *cloud_in, cloud, tfListener_);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcrgb);

    int c_w = cloud_in->width / 2;
    int c_h = cloud_in->height / 2;

    auto point_3d = pcrgb->at(c_w,c_h);
    x = point_3d.x;
    y = point_3d.y;
    z = point_3d.z;

    std::cout << "(" << x << "," << y << "," << z << ")" << std::endl;
  }

private:
  
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  tf::TransformListener tfListener_;

};

void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }

void set_goal(move_base_msgs::MoveBaseGoal& goal, char* arg)
  {
    float x,y;
    ROS_INFO("ARG = %s\n",arg);
    
;

    if(arg == "room1")
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
    else if (arg == "hall")
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
      ROS_INFO("NOTHING RECEIVED\n");
    }

    goal.target_pose.pose.orientation.w = 0.0013;
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_center");
  RGBDFilter rf;

  
  move_base_msgs::MoveBaseGoal goal;

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  goal.target_pose.header.frame_id = "map";

  set_goal(goal, argv[argc-1]);
  goal.target_pose.header.stamp = ros::Time::now();
  ROS_INFO("Sending goal");
  ac.sendGoal(goal,doneCb);
  ac.waitForResult();
  

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      ROS_INFO("Hooray, mission accomplished");
  }
  else
  {
      ROS_INFO("[Error] mission could not be accomplished");
  }
  

  ros::spin();
  return 0;
}