#include "Behavior_hospital/Find_object.h"

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
namespace behavior_hospital_bica
{

FindObject::FindObject()
{
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &FindObject::cloudCB, this);
}

FindObject::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
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

void
FindObject::step()
{
    if(!isActive()){
        return;
    }
    geometry_msgs::Twist msg;

    msg.linear.x = 0.5;

    vel_pub_.publish(msg);
}


} // behavior_hospital_bica