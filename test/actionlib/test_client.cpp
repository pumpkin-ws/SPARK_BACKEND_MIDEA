#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <spark_backend/RobotServiceAction.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "client");
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("simple", true);
    ROS_INFO("waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("server connected, now sending goal");
    spark_backend::RobotServiceGoal goal;
    goal.goal = 100;
    ac.sendGoal(goal);
    sleep(10);
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl;
    
    ac.cancelGoal();
    sleep(2);
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl;
    return EXIT_SUCCESS;
}