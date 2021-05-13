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
#include "geometry_msgs/Twist.h"


class RGBDFilter
{
public:
  RGBDFilter() : buffer_(), listener_(buffer_)
  {
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
    object_sub_ = nh_.subscribe("/detected", 1, &RGBDFilter::calculatePoint2D, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    try
    {
      pcl_ros::transformPointCloud("camera_link", *cloud_in, cloud_point_, tfListener_);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

  }

  void calculatePoint2D(const darknet_ros_msgs::BoundingBox::ConstPtr& objmsg)
  {
      int coor2dx_ = (objmsg->xmin + objmsg->xmax) / 2;
      int coor2dy_ = (objmsg->ymin + objmsg->ymax) / 2;
      object_ = objmsg->Class;

      ROS_INFO("COORDENADAAAAASSSS: (%d, %d)", coor2dx_, coor2dy_);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(cloud_point_, *pcrgb);

      if ((std::isnan(coor2dx_) || std::isnan(coor2dy_)) || !(pcrgb))
      {
        return;
      }
        auto point3d = pcrgb->at(coor2dx_, coor2dy_);
      
        //ROS_INFO("PUNTOS: (%f, %f, %f)", point3d.x, point3d.y, point3d.z);
        //ROS_INFO("%f",point3d.x);
        publish_transform(point3d.x, point3d.y, point3d.z);

  }

  void publish_transform(const float x, const float y, const float z)
  {

    geometry_msgs::TransformStamped odom2bf_msg;
    try
    {
        odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
    }
    catch (std::exception & e)
    {
        return;
    }

    tf2::Stamped<tf2::Transform> bf2obj;
    bf2obj.setOrigin(tf2::Vector3(x, y, z));
    bf2obj.setRotation(tf2::Quaternion(0, 0, 0, 1));
    
    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);
    
    tf2::Transform odom2obj = odom2bf * bf2obj;

    geometry_msgs::TransformStamped map2obj_msg;
    map2obj_msg.header.frame_id = "map";
    map2obj_msg.child_frame_id = object_;
    map2obj_msg.header.stamp = ros::Time::now();
    map2obj_msg.transform = tf2::toMsg(odom2obj);
    tfBroadcaster_.sendTransform(map2obj_msg);
    //step(x);
  }

/*
void step(const float x)
{
  bool put_transform = true;
  double angle;
  geometry_msgs::Twist msg2;
  geometry_msgs::TransformStamped bf2object_2_msg;

  try
  {
      bf2object_2_msg = buffer_.lookupTransform("base_footprint", "odom", ros::Time(0));
  }
  catch (std::exception & e)
  {
      ROS_INFO("SI ENTRA MUY MALOOOOOOOOOOOC\n");
      put_transform = false;
  }
  if(put_transform && (x > 1))
  {
    
    angle = atan2(bf2object_2_msg.transform.translation.y, bf2object_2_msg.transform.translation.x);
    ROS_INFO ("angulo de la transformada = %f ", angle);
    ROS_INFO("istancia al objeto = %f", x);
    if (angle > 1)
    {
        msg2.angular.z = 0.1;
    }
    else if (angle < -1)
    {
        msg2.angular.z = -0.1;
    }
    else
    {
      msg2.linear.x = 0.15;
      msg2.angular.z = 0.0;
      if (x <= 1)
      {
        ROS_INFO("Pa alante");
          msg2.linear.x = 0.0;
          msg2.angular.z = 0.0;
      }
    }
  }

  vel_pub_.publish(msg2);

}
*/

private:
  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber object_sub_;
  ros::Publisher vel_pub_;

  tf2_ros::Buffer buffer_;
  tf2_ros::StaticTransformBroadcaster tfBroadcaster_;
  tf2_ros::TransformListener listener_;
  tf::TransformListener tfListener_;

  std::string object_;

  sensor_msgs::PointCloud2 cloud_point_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_center");
  RGBDFilter rf;

  ros::spin();
  return 0;
}
