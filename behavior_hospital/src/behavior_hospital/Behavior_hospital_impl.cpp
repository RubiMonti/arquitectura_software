#include "behavior_hospital/Behavior_hospital_impl.h"

#include "std_msgs/Bool.h"
#include "ros/ros.h"
namespace behavior_hospital_bica
{

BehaviorHospitalImpl::BehaviorHospitalImpl()
{
    obstacle_sub_ = nh_.subscribe("/obstacle", 1, &BehaviorHospitalImpl::obstacle_callback, this);
}

void
BehaviorHospitalImpl::obstacle_callback(const std_msgs::Bool::ConstPtr& msg)
{
    is_obstacle_ = msg->data;
}


bool 
BehaviorHospitalImpl::Find_object_2_Go_goal()
{

    return (ros::Time::now() - state_ts_).toSec() > 2.0;
}
bool 
BehaviorHospitalImpl::Go_goal_2_Find_object()
{
    return arrived_;
}


} // behavior_hospital_bica