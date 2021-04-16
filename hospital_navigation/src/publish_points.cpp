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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <ros/console.h>

class pointPublisher{

public:

	pointPublisher(): goal_state(0), index_points(0), status_navigation(0)
	{
		wp_pub = node_go_to_wp.advertise<geometry_msgs::PoseStamped>("/navigate_to", 1);
		mv_pub = node_go_to_wp.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		actual_point = node_go_to_wp.subscribe("/move_base/feedback", 1, &pointPublisher::messageCallback, this);
		goal_status = node_go_to_wp.subscribe("/move_base/result", 1,  &pointPublisher::messageStatus, this);

	}

	void turningCallback(const std_msgs::Empty::ConstPtr& finish_turning){ 
		index_points++;
		status_navigation = 0;
	}


	void messageCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& wp_status)
	{
		  ROS_INFO("current location: x: [%lf], y: [%lf], z: [%lf]", wp_status->feedback.base_position.pose.position.x,
	   				wp_status->feedback.base_position.pose.position.y,  wp_status->feedback.base_position.pose.position.y);
		  printf("\n");
	}


	void messageStatus(const move_base_msgs::MoveBaseActionResult::ConstPtr& wp_base_status){
		  ROS_INFO("status navigation: [%u]", wp_base_status->status.status);
		  status_navigation = wp_base_status->status.status;
	}

	void wp_engine(){
		geometry_msgs::PoseStamped point;
		geometry_msgs::Twist velocity_msg;

		switch (status_navigation) 
        {
			case 0:
				ROS_INFO("PUBLISH POINT --->");
				point.pose.position.x = points[index_points][0];
				point.pose.position.y = points[index_points][1];
				point.pose.position.z = points[index_points][2];
				point.pose.orientation.w = 1.0;
				wp_pub.publish(point);

			break;
			case 1:break;
			case 2: ROS_INFO("KOBUKI APROACHING THE GOAL --->");
                velocity_msg.linear.x = 0.1;
                velocity_msg.angular.z = 0.0;
            break;
			case 3:
				ROS_INFO("GOAL REACHED!");
				status_navigation = 5;
                velocity_msg.linear.x = 0.0;
                velocity_msg.angular.z = 0.0;
			break;
			case 4:ROS_ERROR("MISSION ABORTED");break;
			case 5: break;
		}

        mv_pub.publish(velocity_msg);
	}

	int get_status_navigation()
    {
        return status_navigation;
    }
	int get_index_points()
    { 
        return index_points;
    }
	int get_total_points()
    {
        return total_points;
    }

private:
	ros::NodeHandle node_go_to_wp;

	int goal_state; 		   			
	int index_points; 					
	static const int total_points = 2; 
	int status_navigation; 				

// ------- HOSPITAL MAP ------- //

    const float start_goal[3] = {0.0, 0.0, 0.0};
	const float first_position[3] = {2.0, 2.0, 1.0}; 
	
	const float *points[total_points] = {first_position, start_goal};	//final array of points(coordinates)

	ros::Time turn_ts_; 

	ros::Publisher wp_pub;
	ros::Publisher mv_pub; 
	ros::Subscriber actual_point;
	ros::Subscriber goal_status;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "points_publication");

	pointPublisher go_to_point;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ROS_WARN("index_points: %d | total_points: %d", go_to_point.get_index_points(), go_to_point.get_total_points());
		if (go_to_point.get_index_points() == go_to_point.get_total_points()){
			ROS_INFO("DONE");
			exit(0);
		}

		go_to_point.wp_engine();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
