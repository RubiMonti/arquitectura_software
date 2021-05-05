#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>
#include <pcl_ros/transforms.h>

#include "darknet_ros_msgs/BoundingBoxes.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

class DarknetDetection
{
public:
    DarknetDetection()
    {
        object_detection_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &DarknetDetection::objectCallback, this);
        finish_detection_ = nh_.advertise<std_msgs::Bool>("/detected", 1);
    }

    void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
    {
        std_msgs::Bool object_detected_;     

		int size = box_msg->bounding_boxes.size();
	
        for(int iter = 0; iter <= size; iter++)
        {
            if (box_msg->bounding_boxes[iter].Class == "chair" && box_msg->bounding_boxes[iter].probability > 0.35)
            {
                ROS_INFO("OBJECT DETECTED!");
                finish_detection_.publish(object_detected_);

            }   
        }
		
	}

private:
    ros::NodeHandle nh_;

    ros::Subscriber object_detection_;
    ros::Publisher finish_detection_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet");

    DarknetDetection dd;

    ros::Rate loop_rate(20);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}