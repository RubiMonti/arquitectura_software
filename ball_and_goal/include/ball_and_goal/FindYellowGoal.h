#ifndef BALL_AND_GOAL__FIND_BLUE_GOAL_H__
#define BALL_AND_GOAL__FIND_BLUE_GOAL_H__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include <string>

namespace ball_and_goal_bica
{

class FindYellowGoal : public bica::Component
{
public:
    FindYellowGoal();
    void imageCb(const sensor_msgs::Image::ConstPtr& msg);
    void publish_detection(float x, float y);
    void step();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    int x_ = 0;
    int y_ = 0;
    int counter_ = 0;
};

} // ball_and_goal

#endif // BALL_AND_GOAL__FIND_BLUE_GOAL_H__