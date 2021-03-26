#include "ball_and_goal/FindBlueGoal.h"


#include "geometry_msgs/Twist.h"
#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ball_and_goal_bica
{

FindBlueGoal::FindBlueGoal() : it_(nh_) , buffer_() , listener_(buffer_) 
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FindBlueGoal::imageCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

// double
// FindBall::publish_detection(float x, float y)
// {
//     double angle;
//     float x = 1;
//     float y = 0;

//     geometry_msgs::TransformStamped odom2bf_msg;
//     try{
//         odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
//     }   catch (std::exception & e)
//     {
//         return;
//     }

//     tf2::Stamped<tf2::Transform> odom2bf;
//     tf2::fromMsg(odom2bf_msg, odom2bf);

//     tf2::Stamped<tf2::Transform> bf2object;
//     bf2object.setOrigin(tf2::Vector3(x, y ,0));
//     bf2object.setRotation(tf2::Quaternion(0, 0, 0, 1));

//     tf2::Transform odom2object = odom2bf * bf2object;

//     geometry_msgs::TransformStamped odom2object_msg;
//     odom2object_msg.header.stamp = ros::Time::now();
//     odom2object_msg.header.frame_id = "odom";
//     odom2object_msg.child_frame_id = "object";

//     odom2object_msg.transform = tf2::toMsg(odom2object);

//     broadcaster.sendTransform(odom2object_msg);

//     geometry_msgs::TransformStamped bf2obj_2_msg;
//     try {
//         bf2obj_2_msg = buffer_.lookupTransform( "base_footprint", "object", ros::Time(0));
//     } catch (std::exception & e)
//     {
//         return;
//     }

//     //angulo del robot respecto a la pelota
//     angle = atan2(bf2obj_2_msg.transform.translation.y, bf2obj_2_msg.transform.translation.x);
//     return angle;

// }

void
FindBlueGoal::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
    //if(!isActive()){
    //    return;
    //}

    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//imagen que me acaba de llegar

    cv::Mat hsv;
    cv:cvtColor(cv_ptr->image , hsv, CV_RGB2HSV);

    int height = hsv.rows;
    int width = hsv.cols;
    int step = hsv.step;
    int channels = 3;  // RGB

    x_ = 0;
    y_ = 0;
    counter_ = 0;
    for (int i=0; i < height; i++ ){
        for (int j=0; j < width; j++ )
        {
            int posdata = i * step + j * channels;
            
            if((hsv.data[posdata] >= 0) && (hsv.data[posdata] <= 153) && (hsv.data[posdata+1]  >= 176) && (hsv.data[posdata+1] <= 223) && (hsv.data[posdata+2]  >=70) && (hsv.data[posdata+2] <= 92))
            {
                x_ += j;
                y_ += i;
                counter_++;
            } 
        }
    }
}

void
FindBlueGoal::step(){
    //if(!isActive()){
    //    return;
    //}
    geometry_msgs::Twist msg2;

    int pos_x,pos_y;
    if (counter_ > 500){
        ROS_INFO("\nGoal at %d %d\n", x_ / counter_ , y_ / counter_);
        pos_x = x_ / counter_;
        pos_y = y_ / counter_;
       
        msg2.angular.z = 0.2;
        if(pos_x >= 200 && pos_x <= 300)
        {
            /*
            if (pos_y <= 100)
            {
                msg2.linear.x = 0.0;
            }
            else 
            {
                msg2.linear.x = 0.2;
            }
            */
            msg2.linear.x = 0.2;
            msg2.angular.z = 0.0;
        }
    }
    else {
        ROS_INFO("\nNO GOAL FOUND\n");
        msg2.angular.z = 0.5;

    }
    vel_pub_.publish(msg2);

}

} // ball_and_goal

