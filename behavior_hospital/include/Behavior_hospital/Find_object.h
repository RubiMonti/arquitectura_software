#ifndef BEHAVIOR_HOSPITAL_FIND_OBJECT_H__
#define BEHAVIOR_HOSPITAL_FIND_OBJECT_H__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
namespace behavior_hospital_bica
{

class FindObject : public bica::Component
{
public:
    FindObject();
    void step();

    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

private:
    ros::NodeHandle nh_;
    ros::Suscriber cloud_sub_;
    tf::TransformListener tfListener_;


};

} // behavior_hospital_bica

#endif // BEHAVIOR_HOSPITAL_FIND_OBJECT_H__