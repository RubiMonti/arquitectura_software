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

#include "behavior_hospital/rgbd_distance.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

namespace behavior_hospital
{

RGBDDistance::RGBDDistance(const std::string& name)
: BT::ActionNodeBase(name, {}), done_(false)
{
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDDistance::cloudCB, this);
    object_sub_ = nh_.subscribe("/detected", 1, &RGBDDistance::calculatePoint2D, this);
}

void
RGBDDistance::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
    sensor_msgs::PointCloud2 cloud;

    float coor3dx, coor3dy, coor3dz;
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

    auto point3d = pcrgb->at(coor2dx_, coor2dy_);
    if (std::isnan(point3d.x) || std::isnan(point3d.y) || std::isnan(point3d.z))
    {
        return;
    }
    coor3dx = point3d.x;
    coor3dy = point3d.y;
    coor3dz = point3d.z;
    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(coor3dx, coor3dy, coor3dz));
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
    done_ = true;
}

void
RGBDDistance::calculatePoint2D(const darknet_ros_msgs::BoundingBox::ConstPtr& objmsg)
{
    coor2dx_ = (objmsg->xmin + objmsg->xmax)/2;
    coor2dy_ = (objmsg->ymin + objmsg->ymax)/2;
    object_ = objmsg->Class;
}

void
RGBDDistance::halt()
{
  ROS_INFO("RGBDDistance halt");
}

BT::NodeStatus
RGBDDistance::tick()
{
  ROS_INFO("RGBDDistance tick");

  if (done_)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace behavior_hospital
