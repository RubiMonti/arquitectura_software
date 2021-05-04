#ifndef BEHAVIOR_HOSPITAL_GO_GOAL_H__
#define BEHAVIOR_HOSPITAL_GO_GOAL_H__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
namespace behavior_hospital_bica
{

class GoGoal : public bica::Component
{
public:
    GoGoal();
    void step();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;

};

} // behavior_hospital_bica

#endif // BEHAVIOR_HOSPITAL_GO_GOAL_H__