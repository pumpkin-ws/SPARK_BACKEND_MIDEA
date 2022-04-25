#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/RobotServiceAction.h"
#include "common.hpp"
#include <unistd.h>

int main(int argc, char** argv) {
    /**
     * @brief This is an example for initializing the action client and waiting for the server
     * 
     */
    ros::init(argc, argv, "robot_client");
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("robot", true);
    ROS_INFO("Waiting for the server to start...");
    ac.waitForServer();
    ROS_INFO("Server connected, now sending goal!\n");
    
    /**
     * @brief This is an example for reading the output
     * 
     */
    spark_backend::RobotServiceGoal goal;
    goal.move_type = static_cast<int>(MoveType::GET_DO);    
    goal.io_type = static_cast<int>(IOType::DO);
    goal.pin_num = 1;
    ac.sendGoal(goal);
    ac.waitForResult();
    spark_backend::RobotServiceResult::ConstPtr result = ac.getResult();
    printf("The state of DO pin 1 is:");
    std::cout << result->io_state << std::endl;

    /**
     * @brief This is an example for reading the DI
     * 
     */
    goal.move_type = static_cast<int>(MoveType::GET_DI);    
    goal.io_type = static_cast<int>(IOType::DI);
    goal.pin_num = 8;
    ac.sendGoal(goal);
    ac.waitForResult();
    result = ac.getResult();
    printf("The state of DI pin %d is:", (int)goal.pin_num);
    std::cout << result->io_state << std::endl;

    /**
     * @brief this is an example for setting the global ros parameter with the node handle
     * 
     */

    ros::NodeHandle nh;
    nh.setParam("TASK_STATE", 1);
    int task_state;
    nh.getParam("TASK_STATE", task_state);

    printf("The current task state is: %.1d \n", task_state);

    /**
     * @brief This is the example for setting the output of the robot
     * 
     */
    goal.move_type = static_cast<int>(MoveType::SET_DO);
    goal.pin_num = 16;
    goal.DO_value = 1;
    goal.io_type = static_cast<int>(IOType::DO);
    ac.sendGoal(goal);
    ac.waitForResult();

    /**
     * @brief This is the example for reading the robot status
     * 
     */
    goal.move_type = static_cast<int>(MoveType::GET_ROBOT_STATE);
    ac.sendGoal(goal);
    ac.waitForResult();
    result = ac.getResult();
    RobotState rs;
    rs = static_cast<RobotState>(result->robot_state);
    switch (rs) {
        case RobotState::RobotPaused : {
            printf("PAUSED\n");
            break;
        }
        case RobotState::RobotResumed : {
            printf("RESUMED\n");
            break;
        }
        case RobotState::RobotRunning : {
            printf("RUNNING\n");
            break;
        }
        case RobotState::RobotStopped : {
            printf("STOPPED\n");
            break;
        }
    }


    return EXIT_SUCCESS;
}