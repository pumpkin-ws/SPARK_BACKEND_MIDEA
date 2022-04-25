#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/ServiceManagerAction.h"
#include "spark_backend/RobotServiceAction.h"
#include "common.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "eih_calib_client");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<spark_backend::ServiceManagerAction> ac("spark_manager", true);
    ROS_INFO("Connecting to service manager...");
    bool connected = ac.waitForServer();
    if (connected == false) {
        std::cout << "Unable to connect to service manager, quitting program." << std::endl;
        exit(-1);
    } else {
        std::cout << "Connected to service manager. " << std::endl;
        // continues to the rest of the program
    }
    spark_backend::ServiceManagerGoal goal;
    goal.manager_task_type = static_cast<int>(ManagerTasks::EIH_CALIBRATION);
    ac.sendGoal(goal);
    ROS_DEBUG("Goal sent, waiting for return from the server.");
    bool returned = ac.waitForResult();
    ROS_DEBUG("Result received, unparsing and displaying the result.");
    auto result = ac.getResult();

    return EXIT_SUCCESS;
}