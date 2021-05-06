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

#include "darknet_ros_msgs/BoundingBoxes.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RGBDFilter
{
public:
  RGBDFilter()
  {
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
    object_sub_ = nh_.subscribe("/detected", 1, &RGBDFilter::calculatePoint2D, this);
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

    auto point3d = pcrgb->at(coor2dx_,coor2dy_);
    coor3dx_ = point3d.x;
    coor3dy_ = point3d.y;
    coor3dz_ = point3d.z;

    //std::cout << "(" << coor3dx_ << "," << coor3dy_ << "," << coor3dz_ << ")" << std::endl;

    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(coor3dx_, coor3dy_, coor3dz_));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transform.stamp_ = ros::Time::now();
    transform.frame_id_ = "/base_footprint";
    transform.child_frame_id_ = object_;

    try
    {
      tfBroadcaster_.sendTransform(transform);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

  }

  void calculatePoint2D(const darknet_ros_msgs::BoundingBox::ConstPtr& objmsg)
  {
    coor2dx_ = (objmsg->xmin + objmsg->xmax)/2;
    coor2dy_ = (objmsg->ymin + objmsg->ymax)/2;
    object_ = objmsg->Class;
    std::cout << "(" << coor2dx_ << "," << coor2dy_ << ")" << std::endl;
  }



private:
  
  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber object_sub_;

  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;

  std::string object_;

  float coor2dx_;
  float coor2dy_;
  float coor3dx_;
  float coor3dy_;
  float coor3dz_;

};

//void doneCb(const actionlib::SimpleClientGoalState& state,
//            const move_base_msgs::MoveBaseResultConstPtr& result)
//  {
//    ROS_INFO("Finished in state [%s]", state.toString().c_str());
//  }

//void set_goal(move_base_msgs::MoveBaseGoal& goal, char* arg)
//  {
//    float x,y;
//    ROS_INFO("ARG = %s\n",arg);
//    
//    x = goal.target_pose.pose.position.x;
//    y = goal.target_pose.pose.position.y;
//
//    if(!(strcasecmp(arg, "room1")))
//    {
//      ROS_INFO("Going to room1\n");
//      x = -6.13;
//      y = 8.2;
//    }
//    else
//    {
//      ROS_INFO("NOTHING RECEIVED\n");
//    }
//
//    goal.target_pose.pose.orientation.w = 1;
//    
//  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_center");
  RGBDFilter rf;

  /*
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
  */
  ros::spin();
  return 0;
}