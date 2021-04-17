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

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>

namespace hospital_navigation
{
class Navigation
{
  public:
    Navigation(ros::NodeHandle& nh) : nh_(nh), action_client_("/move_base", false), goal_sent_(false), action(START)
    {
      wp_sub_ = nh_.subscribe("/navigate_to", 1, &Navigation::navigateCallback, this);
    }

    void navigateCallback(geometry_msgs::PoseStamped goal_pose_)
    {
      ROS_INFO("[navigate_to_wp] Commanding to (%f %f)", goal_pose_.pose.position.x, goal_pose_.pose.position.y);
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = goal_pose_;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      action_client_.sendGoal(goal);
      goal_sent_ = true;
    }

    void step()
    {
		//-------------------------------------------------------------
		switch(action){
			case START:
			//Segun inicio empiezo la navegacion
				action = MOVE;
			break;
			case MOVE:
				if(goal_sent_){ 
					action = RETURN_TO_BASE;
				}
			break;
		}

		//---------------------------------------------------------------
      if (goal_sent_)
      {
          actionlib::SimpleClientGoalState state = action_client_.getState();
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("GOAL REACHED!");
          else
            ROS_INFO("Something bad happened! :((((( ");
          goal_sent_ = false;
      }
    }

  private:
        static const int START = 0;
        static const int MOVE = 1;
        static const int RETURN_TO_BASE = 2;

        int action;

        ros::NodeHandle nh_;
        ros::Subscriber wp_sub_;
        bool goal_sent_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;

};
}  // namespace hospital_navigation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_kobuki");
  ros::NodeHandle nh("~");
  hospital_navigation::Navigation kobuki_navigation(nh);
  while (ros::ok())
  {
    kobuki_navigation.step();
    ros::spinOnce();
  }
  return 0;
}
