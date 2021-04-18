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
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



void set_goal(move_base_msgs::MoveBaseGoal& goal, char* arg)
{
    ROS_INFO("ARG = %s\n", arg);
    if (!(strcasecmp(arg,"habitacion1")))
    {
        ROS_INFO("Mandando a habitacion 1\n");
        goal.target_pose.pose.position.x = -6.13;
        goal.target_pose.pose.position.y = 8.2;
    }
    else if (!(strcasecmp(arg,"habitacion2")))
    {
        ROS_INFO("Mandando a habitacion 2\n");
        goal.target_pose.pose.position.x = -5.97;
        goal.target_pose.pose.position.y = -8.21;
    }
    else if (!(strcasecmp(arg,"spawn")))
    {
        ROS_INFO("Mandando a spawn\n");
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
    }
    else if (!(strcasecmp(arg,"consulta1")))
    {
        ROS_INFO("Mandando a consulta 1\n");
        goal.target_pose.pose.position.x = -5.37;
        goal.target_pose.pose.position.y = 1.13;
    }
    else if (!(strcasecmp(arg,"consulta2")))
    {
        ROS_INFO("Mandando a consulta 2\n");
        goal.target_pose.pose.position.x = -5.22;
        goal.target_pose.pose.position.y = -1.37;
    }
    else if (!(strcasecmp(arg,"almacen1")))
    {
        ROS_INFO("Mandando a almacen 1\n");
        goal.target_pose.pose.position.x = 12.5;
        goal.target_pose.pose.position.y = 7.89;
    }
    else if (!(strcasecmp(arg,"almacen2")))
    {
        ROS_INFO("Mandando a almacen 2\n");
        goal.target_pose.pose.position.x = 12.47;
        goal.target_pose.pose.position.y = -7.72;
    }
    else
        ROS_INFO("No se ha detectado nada\n");
    
    goal.target_pose.pose.orientation.w = 0.0013;

}

void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}


int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");

    move_base_msgs::MoveBaseGoal goal;

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){

        ROS_INFO("Waiting for the move_base action server to come up");

    }

    goal.target_pose.header.frame_id = "map";

    set_goal(goal, argv[argc - 1]);


    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal");
    ac.sendGoal(goal, doneCb);//, MoveBaseClient::SimpleActiveCallback(), feedbackCb);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

        ROS_INFO("Hooray, the base moved 1 meter forward");

    else

        ROS_INFO("The base failed to move towards the desired point for some reason");



    return 0;

}