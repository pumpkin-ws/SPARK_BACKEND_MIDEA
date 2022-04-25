/**
 * @file move_robot.cpp
 * @author your name (you@domain.com)
 * @brief This is a client that attempts to move robot with keyboard presses
 * @version 0.1
 * @date 2021-03-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/RobotServiceAction.h"
#include <thread>
#include <vector>
#include "common.hpp"
#include <cmath>
#include "spark_backend/AuboInfo.h"
#include <termio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mutex>

int keyboardScan(){
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

struct WayPointMsg {
    /**
     * @brief way points will be stored as
     * x, y, z, w, rx, ry, rz
     * the rotation will be in quaternion form
     * 
     */
    std::vector<float> way_points;
    std::vector<float> current_joints;
    std::mutex mtx;
};

struct CaptureSignal {
    int capture_signal{-1};
    std::mutex mtx;
};

void auboMsgCallback(const spark_backend::AuboInfo::ConstPtr &msg, WayPointMsg *wpm){
    // lock_guard is very suitable to use in a callback function, as the lock_guard object will automatically release object at 
    // the end of the call back execution!
    std::lock_guard<std::mutex> lock (wpm->mtx);
    wpm->way_points.clear();
    wpm->way_points.insert(
        wpm->way_points.end(), 
        msg->waypoint_values.begin(), 
        msg->waypoint_values.end()
    );  
    wpm->current_joints.clear();
    wpm->current_joints.insert(
        wpm->current_joints.end(),
        msg->joint_values.begin(),
        msg->joint_values.end()
    );
}

void wayPointSubscriber(WayPointMsg *wpm, CaptureSignal *cs) {
    ros::NodeHandle nh;
    ros::Subscriber waypoint_subscriber = nh.subscribe<spark_backend::AuboInfo>("aubo_info", 100, boost::bind(&auboMsgCallback, _1, wpm));
    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        cs->mtx.lock();
        if (cs->capture_signal == 27) {
            cs->mtx.unlock();
            std::cout << "\nExiting thread way point subscriber..." << std::endl;
            break;
        } 
        cs->mtx.unlock();
        // wpm->mtx.lock();
        // std::cout << "The size of the way point message vector is " << wpm->way_points.size() << std::endl;
        // wpm->mtx.unlock();
    }
}

void kbListener(CaptureSignal *cs) {
    while(true) {
        // std::cout << "Waiting for keyboard signal!" << std::endl;
        cs->capture_signal = keyboardScan(); 
        sleep(0.1);
        cs->mtx.lock();
        if(cs->capture_signal == 27) {
            cs->mtx.unlock();
            std::cout << "\nExiting thread keyboard listener..." << std::endl;
            break;
        }
        cs->mtx.unlock();
    }
}

void robotClient(CaptureSignal *cs, WayPointMsg *wpm) {
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("robot", true); 
    ROS_INFO("Waiting for the server to start...");
    ac.waitForServer();
    ROS_INFO("Server started, able to accept goal now!");
    spark_backend::RobotServiceGoal goal;
    while(true) {
        sleep(0.5);
        //std::cout << "The captured key signal is " << cs->capture_signal << std::endl;
        cs->mtx.lock();
        if (cs->capture_signal == 27) {
            std::cout << "\nExiting thread robot client..." << std::endl;
            cs->mtx.unlock();
            nh.shutdown();
            break;
        } 
        switch(cs->capture_signal) {
            case 113 : {
                printf("\nq is pressed.\n");
                printf("The robot will move in the positive x direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                std::vector<float> cur_joints = wpm->current_joints;
                wpm->mtx.unlock();
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                std::vector<float> target_pose = cur_pose;
                if (target_pose.size() != 0 && cur_joints.size() != 0) {
                    // add an offset in the positive x direction
                    target_pose[0] += 0.1;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];

                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    
                    std::cout << "The angles are: " << std::endl;
                    std::cout << target_pose_angles[2] * 180 / M_PI << std::endl;
                    std::cout << target_pose_angles[1] * 180 / M_PI << std::endl;
                    std::cout << target_pose_angles[0] * 180 / M_PI << std::endl;

                    goal.current_joints[0] = cur_joints[0];
                    goal.current_joints[1] = cur_joints[1];
                    goal.current_joints[2] = cur_joints[2];
                    goal.current_joints[3] = cur_joints[3];
                    goal.current_joints[4] = cur_joints[4];
                    goal.current_joints[5] = cur_joints[5];

                    goal.use_joint = false;
                    goal.set_fraction = true;
                    goal.move_rate = 0.1;
                    // goal.move_type = int(MoveType::MOVE_J);
                    goal.move_type = int(MoveType::MOVE_L);
                    
                    ac.sendGoal(goal);
                }

                break;
            }
            case 97 : {
                printf("\na is pressed.\n");
                printf("The robot will move in the negative x direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                wpm->mtx.unlock();
                std::vector<float> target_pose = cur_pose;
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                if (target_pose.size() != 0) {
                    // add an offset in the positive x direction
                    target_pose[0] -= 0.5;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];
                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    goal.use_joint = false;
                    goal.set_fraction = true;
                    goal.move_rate = 0.1;
                    goal.move_type = int(MoveType::MOVE_L);
                    ac.sendGoal(goal);
                }                
                break;                
            }
            case 119 : {
                printf("\nw is pressed.\n");
                printf("The robot will move in the positive y direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                wpm->mtx.unlock();
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                std::vector<float> target_pose = cur_pose;
                if (target_pose.size() != 0) {
                    // add an offset in the positive x direction
                    target_pose[1] += 0.5;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];
                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    goal.use_joint = false;
                    goal.move_type = int(MoveType::MOVE_L);
                    ac.sendGoal(goal);
                }  
                break;                  
            }
            case 115 : {
                printf("\ns is pressed.\n");
                printf("The robot will move in the negative y direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                wpm->mtx.unlock();
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                std::vector<float> target_pose = cur_pose;
                if (target_pose.size() != 0) {
                    // add an offset in the positive x direction
                    target_pose[1] -= 0.5;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];
                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    goal.use_joint = false;
                    goal.move_type = int(MoveType::MOVE_L);
                    ac.sendGoal(goal);
                } 
                break;                   
            }
            case 101 : {
                printf("\ne is pressed.\n");
                printf("The robot will move in the positive z direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                wpm->mtx.unlock();
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                std::vector<float> target_pose = cur_pose;
                if (target_pose.size() != 0) {
                    // add an offset in the positive x direction
                    target_pose[2] += 0.5;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];
                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    goal.use_joint = false;
                    goal.move_type = int(MoveType::MOVE_L);
                    ac.sendGoal(goal);
                } 
                break;                  
            }
            case 100 : {
                printf("\nd is pressed.\n");
                printf("The robot will move in the negative z direction!\n");
                cs->capture_signal = -1;
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                wpm->mtx.unlock();
                printf("The current pose values are: \n");
                printf("--------------------------------\n");
                for (int i = 0; i < cur_pose.size(); i++) {
                    std::cout << cur_pose[i] << std::endl;
                }
                std::vector<float> target_pose = cur_pose;
                if (target_pose.size() != 0) {
                    // add an offset in the negative y direction
                    target_pose[2] -= 0.5;
                    // construct the euler angles
                    Eigen::Quaternionf rpose;
                    rpose.w() = target_pose[3];
                    rpose.x() = target_pose[4];
                    rpose.y() = target_pose[5];
                    rpose.z() = target_pose[6];

                    Eigen::Vector3f target_pose_angles = rpose.matrix().eulerAngles(2, 1, 0);
                    goal.move_line_pos[0] = target_pose[0];
                    goal.move_line_pos[1] = target_pose[1];
                    goal.move_line_pos[2] = target_pose[2];
                    goal.move_line_pos[3] = target_pose_angles[2];
                    goal.move_line_pos[4] = target_pose_angles[1];
                    goal.move_line_pos[5] = target_pose_angles[0];
                    goal.use_joint = false;
                    goal.move_type = int(MoveType::MOVE_L);
                    ac.sendGoal(goal);
                } 
                break;                  
            }
            case 116 : {
                cs->capture_signal = -1;
                printf("\nKey t is pressed, now the move track joint will be tested.\n");
                goal.move_type = static_cast<int>(MoveType::TRACK_JOINT);
                // get the current robot pose
                wpm->mtx.lock();
                std::vector<float> cur_pose = wpm->way_points;
                std::vector<float> cur_joints = wpm->current_joints;
                wpm->mtx.unlock();
                
                int track_pos_num = 3;
                goal.num_of_track_points = track_pos_num;
                std::vector<double> pos1(6), pos2(6), pos3(6);
                pos1[0] = cur_pose[0] + 0.1;
                pos1[1] = cur_pose[1];
                pos1[2] = cur_pose[2];
                pos1[3] = cur_pose[3];
                pos1[4] = cur_pose[4];
                pos1[5] = cur_pose[5];
                ROS_INFO("Done setting pos1");

                pos2[0] = pos1[0];
                pos2[1] = pos1[1] + 0.1;
                pos2[2] = pos1[2];
                pos2[3] = pos1[3];
                pos2[4] = pos1[4];
                pos2[5] = pos1[5];
                ROS_INFO("Done setting pos2");

                pos3[0] = pos2[0];
                pos3[1] = pos2[1];
                pos3[2] = pos2[2] + 0.1;
                pos3[3] = pos2[3];
                pos3[4] = pos2[4];
                pos3[5] = pos2[5];


                goal.current_joints[0] = cur_joints[0];
                goal.current_joints[1] = cur_joints[1];
                goal.current_joints[2] = cur_joints[2];
                goal.current_joints[3] = cur_joints[3];
                goal.current_joints[4] = cur_joints[4];
                goal.current_joints[5] = cur_joints[5];

                // insert the previously defined three points into the track pos vector
                goal.move_track_pos.insert(goal.move_track_pos.end(), pos1.begin(), pos1.end());
                goal.move_track_pos.insert(goal.move_track_pos.end(), pos2.begin(), pos2.end());
                goal.move_track_pos.insert(goal.move_track_pos.end(), pos3.begin(), pos3.end());
                ROS_INFO("Set positions successfully");

                std::cout << "The size of the track pos vector is " << goal.move_track_pos.size() << std::endl;
                goal.use_joint = false;
                goal.set_fraction = true;
                goal.move_rate = 0.01;
                // goal.move_type = int(MoveType::MOVE_J);
                goal.move_type = int(MoveType::TRACK_JOINT);
                ac.sendGoal(goal);

            }
            case 32 : {
                printf("\nthe space bar is pressed, the previous move command will be canceled.\n");
                cs->capture_signal = -1;
                goal.move_type = static_cast<int>(MoveType::STOP);
                ac.sendGoal(goal);
                break;
            }
            case 112 : {
                printf("\nP was pressed. The robot movement will be paused!\n");
                cs->capture_signal = -1;
                goal.move_type = static_cast<int>(MoveType::PAUSE);
                ac.sendGoal(goal);
                break;
            }
            case 114 : {
                printf("\nr was pressed. The robot movement will be resumed.\n");
                cs->capture_signal = -1;
                goal.move_type = static_cast<int>(MoveType::RESUME);
                ac.sendGoal(goal);
                break;
            }
            case -1: {
                break;
            }
            default : {
                printf("!---------------------------------------------------------!\n");
                std::cout << "The captured key signal is " << cs->capture_signal << std::endl;
                printf("The captured signal is not one of the programmed signal.\n");
                printf("!---------------------------------------------------------!\n");
                break;
            }
        }
        cs->mtx.unlock();
    }        
    }


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_mover");
    WayPointMsg wpm;
    CaptureSignal cs;
    std::thread t_waypoint_subscriber(wayPointSubscriber, &wpm, &cs);
    std::thread t_keyboard_scanner(kbListener, &cs);
    std::thread t_robot_client(robotClient, &cs, &wpm);

    // the threads need to be joined properly to not throw errors, otherwise a segmentation fault will be thrown. 
    if (t_waypoint_subscriber.joinable()) {
         t_waypoint_subscriber.join();
    }
    if (t_keyboard_scanner.joinable()) {
        t_keyboard_scanner.join();
    }
    if (t_robot_client.joinable()) {
        t_robot_client.join();
    }
    return EXIT_SUCCESS;
}

/**
 * @brief exam
 * 
 * @tparam T 
 * @param msg 
 */
template<typename T>
void print(T msg) {
    std::cout << msg << std::endl;
}