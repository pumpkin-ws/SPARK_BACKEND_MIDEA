#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/VisionServiceAction.h"
#include "common.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_client");
    actionlib::SimpleActionClient<spark_backend::VisionServiceAction> ac("vision");
    ROS_INFO("Waiting for vision server to start...");
    ac.waitForServer();
    ROS_INFO("Server started, ready to send goal..");
    spark_backend::VisionServiceGoal goal;
    goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_2D_OBJECT);
    ac.sendGoal(goal);
    ac.waitForResult();
    return EXIT_SUCCESS;
} 