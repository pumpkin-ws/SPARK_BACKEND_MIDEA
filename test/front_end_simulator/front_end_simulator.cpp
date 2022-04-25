#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/ServiceManagerAction.h"
#include "spark_backend/RobotServiceAction.h"
#include "common.hpp"
#include "termio.h"
#include <thread>
#include <mutex>

enum class ControlConstants {
    PAUSE = 112,
    RESUME = 114,
    EXECUTE = 101,
    STOP = 115 //stop will stop the motion of the program, and SHOULD (yet need to test) stop the workflow of the service manager
};

int keyboardScan() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    in = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

struct CaptureSignal {
    int capture_signal{-1};
    std::mutex mtx;
};

void keyboardListener(CaptureSignal *cs) {
    while (true) {
        cs->capture_signal = keyboardScan();
        if (cs->capture_signal == 27) {
            printf("\nDetected the exit key is pressed, will exit the keyboard listener.\n");
            break;
        }
    }
}

/**
 * @brief The robot client will be controlling the pause, stop, and resume of the robot
 * 
 * @param cs 
 */
void robotClient(CaptureSignal *cs) {
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("robot", true);
    ROS_INFO("Waiting for the robot server to start.");
    ac.waitForServer();
    ROS_INFO("The robot server has started. Waiting to receive command.");
    spark_backend::RobotServiceGoal goal;
    
    while (true) {
        // put a sleep here to make up for bad synchronization
        sleep(0.1);
        cs->mtx.lock();
        // breaks out of the while loop if the esc key is detected
        if (cs->capture_signal == 27) {
            printf("\nExiting robot client!\n");
            cs->mtx.unlock();
            break;
        }
        switch (cs->capture_signal) {
            // case 112 : {
            //     printf("\n \"p\"ause is pressed\n");
            //     // call the service to terminate the 
            //     goal.move_type = static_cast<int>(MoveType::STOP);
            //     ac.sendGoal(goal);
            //     printf("Setting ros param to PAUSED\n");
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_PAUSED);
            //     cs->capture_signal = -1;
            //     break;
            // }
            // case 114 : {
            //     printf("\n \"r\"esume is pressed\n");
            //     goal.move_type = static_cast<int>(MoveType::RESUME);
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;
            //     printf("Setting ros param to RUN\n");
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN);
            //     break;
            // }
            // case 115 : {
            //     printf("\n \"s\"top is pressed\n");
            //     goal.move_type = static_cast<int>(MoveType::STOP);
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;
            //     printf("Setting ros param to STOPPED\n");
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_STOPPED);
            //     break;
            // }
            case -1 : {
                break;
            }
            default : {
                break;
            }
        }
        cs->mtx.unlock();
    }


};

/**
 * @brief This function will be starting the client server
 * 
 * @param cs 
 */
void managerClient(CaptureSignal *cs) {
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<spark_backend::ServiceManagerAction> ac("spark_manager", true);
    ROS_INFO("Waiting for manager server to start.");
    ac.waitForServer();
    ROS_INFO("The service manager has started. Waiting to receive command");
    spark_backend::ServiceManagerGoal goal;

    while (true) {
        sleep(0.1);
        cs->mtx.lock();
        if (cs->capture_signal == 27) {
            printf("\nESC key detected, exiting service manager.\n");
            cs->mtx.unlock();
            break;
        }
        switch (cs->capture_signal) {
            // case int('f') : {
            //     printf("\n \"f\"get aubo message is pressed\n");
            //     goal.manager_task_type = static_cast<int>(ManagerTasks::GET_AUBO_INFO);
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN); 
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;  
            //     break;              
            // }
            // case int('j') : {
            //     printf("\n \"j\" Generate close up template is pressed\n");
            //     goal.manager_task_type = static_cast<int>(ManagerTasks::GENERATE_CLOSEUP_TEMPALTE);
            //     goal.PCB_template_name = "6601";
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN); 
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;
            //     break; 
            // }
            // case int('k'): {
            //     printf("\n \"k\" Generate midea pcb template is pressed\n");
            //     goal.manager_task_type = static_cast<int>(ManagerTasks::GENERATE_MIDEA_PCB_2021_TEMPLATE);
            //     goal.PCB_template_name = "7601s";
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN); 
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;  
            //     break;
            // }
            case int('o') : {
                printf("\n \"o\" urgent problems to be tested for today..\n");
                nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN); 
                goal.manager_task_type = static_cast<int>(ManagerTasks::URGENT_TEST);
                goal.PCB_template_name = "7601s";
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            // case int('c') : {
            //     printf("\n \"c\" Start performing midea pcba work flow.\n");
            //     nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_RUN);
            //     goal.manager_task_type = static_cast<int>(ManagerTasks::MIDEA_PCBA);
            //     goal.PCB_template_name = "7601s";
            //     goal.speed_ratio = 0.1;
            //     ac.sendGoal(goal);
            //     cs->capture_signal = -1;
            //     break;
            // }
            case int('a'): {
                printf("\n \"a\" Open gripper.\n");
                goal.manager_task_type = static_cast<int>(ManagerTasks::TOGGLE_GRIPPER);
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            case int('s'): {
                printf("\n \"s\" Toggle rig1 up.\n");
                goal.manager_task_type = static_cast<int>(ManagerTasks::RIG1_UP_TOGGLE);
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            case int('d'): {
               printf("\n \"d\" Toggle rig2 up.\n");
                goal.manager_task_type = static_cast<int>(ManagerTasks::RIG2_UP_TOGGLE);
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            case int('f'): {
               printf("\n \"f\" Toggle rig1 up.\n");
                goal.manager_task_type = static_cast<int>(ManagerTasks::RIG1_DOWN_TOGGLE);
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            case int('g'): {
               printf("\n \"g\" Toggle rig1 up.\n");
                goal.manager_task_type = static_cast<int>(ManagerTasks::RIG2_DOWN_TOGGLE);
                ac.sendGoal(goal);
                cs->capture_signal = -1;
                break;
            }
            case -1 : {
                break;
            }
            default: {
                break;
            }
        }
        cs->mtx.unlock();
    }

};

int main(int argc, char** argv) {
    printf("This program will be initiating both the service manager client and the robot client!\n");
    ros::init(argc, argv, "front_end_simulator");
    CaptureSignal cs;
    std::thread robot_client_thread (robotClient, &cs);
    std::thread service_manager_thread (managerClient, &cs);
    std::thread keyboard_thread (keyboardListener, &cs);
    
    // join the threads to the main thread before the termination of program
    if (robot_client_thread.joinable()) {
        robot_client_thread.join();
    }

    if (service_manager_thread.joinable()) {
        service_manager_thread.join();
    }

    if (keyboard_thread.joinable()) {
        keyboard_thread.join();
    }


    return EXIT_SUCCESS;
}
