#include "behavior_hospital/Go_goal.h"

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
namespace behavior_hospital_bica
{

GoGoal::GoGoal()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
GoGoal::step()
{
    if(!isActive()){
        return;
    }
    geometry_msgs::Twist msg;

    msg.linear.x = 0.5;

    vel_pub_.publish(msg);
}


} // behavior_hospital_bica