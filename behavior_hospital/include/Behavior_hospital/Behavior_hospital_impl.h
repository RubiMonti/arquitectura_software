#ifndef BEHAVIOR_HOSPITAL_BEHAVIOR_HOSPITAL_IMPL_H__
#define BEHAVIOR_HOSPITAL_BEHAVIOR_HOSPITAL_IMPL_H__

#include "bump_and_go_bica.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
namespace behavior_hospital_bica
{

class BehaviorHospitalImpl : public bica::Behavior_hospital
{
public:
    BumpAndGoBicaImpl();

    bool Find_object_2_Go_goal();
    bool Go_goal_2_Find_object();

private:
    ros::NodeHandle nh_;
    // ros::Subscriber obstacle_sub_;

    bool arrived_;

};

} // behavior_hospital_bica

#endif // BEHAVIOR_HOSPITAL_BEHAVIOR_HOSPITAL_IMPL_H__