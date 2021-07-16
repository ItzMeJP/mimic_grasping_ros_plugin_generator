//
// Created by joaopedro on 14/07/21.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <object_recognition_skill_msgs/ObjectRecognitionSkillAction.h>
#include <geometry_msgs/PoseStamped.h>

void chatterCallback(const geometry_msgs::PoseStamped ::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "sub_test");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(0);
    //spinner.start();

    actionlib::SimpleActionClient<object_recognition_skill_msgs::ObjectRecognitionSkillAction> ac("/object_recognition/ObjectRecognitionSkill", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    object_recognition_skill_msgs::ObjectRecognitionSkillGoal goal;
    goal.clusterIndex = -1;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
        ros::Subscriber sub_ = nh.subscribe("/object_recognition/localization_pose", 10, chatterCallback);


    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(2.0); // Timeout of 2 seconds
    while(ros::Time::now() - start_time < timeout) {
        ros::spinOnce();
    }

    //ros::waitForShutdown();
    return 0;
}