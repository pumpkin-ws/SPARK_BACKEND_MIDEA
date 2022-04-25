/**
 * @file spark_service_manager.hpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief The spark service manager
 * @version 0.1
 * @date 2021-04-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SERVICE_MANAGER_HPP_
#define SERVICE_MANAGER_HPP_
// YAML
#include "yaml-cpp/yaml.h"
// ROS
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "spark_backend/ServiceManagerAction.h"
#include "spark_backend/AuboInfo.h"
#include "spark_backend/RobotServiceAction.h"
#include "spark_backend/VisionServiceAction.h"

// STD & Boost
#include <thread>
#include <mutex>
#include <stack>
#include <queue>
#include <boost/filesystem.hpp>
#include <atomic>
#include <future>

// USER DEFINED
#include "math/include/coord_transform.hpp"
#include "io/plc/include/omron_plc.hpp"
#include "common.hpp"
#include "common_signal_chart.hpp"
#include "filesystem/include/file_system.hpp"

namespace spark_backend {

class ServiceManager {
private:
    // TODO: the node handle and the action server
    ros::NodeHandle m_nh;
    actionlib::SimpleActionServer<spark_backend::ServiceManagerAction> m_as;
    spark_backend::ServiceManagerFeedback m_feedback;
    spark_backend::ServiceManagerResult m_result;
    std::string m_action_name;
    ManagerTasks m_manager_tasks;
    int m_robot_num;  // Add by Hudi 2021.05.21
    // TODO: The action client to the robot service
    std::string m_robot_server_name = "robot";
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction>* m_robot_ac_ptr; // the robot action client pointer
    actionlib::SimpleActionClient<spark_backend::VisionServiceAction>* m_vision_ac_ptr;
    // TODO: need to add the robot message subscriber

    /**
     * @brief This is the goal call back, the execution of different 
     * goals are defined here.
     * 
     */
    void goalCB();
    /**
     * @brief preemptCB defines the behavior for when the client 
     * preempts the goal
     * 
     */
    void preemptCB();

    /**
     * @brief unparse the robot work flow files and convert to executable
     * sequences
     * 
     * @param filename 
     */
    void unparseYAMLWorkflow(std::string filename);

    // Delete and Change many functions by Hudi 2021.05.21
    void getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos,
                    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr);
    bool progressBlocker(actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr);

    bool cartesianMoveJointServerCall(
        const std::vector<double> position, 
        const double& speed_ratio,
        actionlib::SimpleActionClient<spark_backend::RobotServiceAction>* ac_ptr);
    bool cartesianMoveLineServerCall(
        const std::vector<double> position, 
        const double& speed_ratio,
        actionlib::SimpleActionClient<spark_backend::RobotServiceAction>* ac_ptr);
    bool moveJointAngleServerCall(
        const std::vector<double> position, 
        const double& speed_ratio,
        actionlib::SimpleActionClient<spark_backend::RobotServiceAction>* ac_ptr);

    int getIO(const IOType io_type, const int pin_num, actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr);
    bool setIO(const int pin_num, const bool state, actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr);

    // void workflowMideaWifi2021();
    // bool wifi2021StateInspection();

    void workflowMideaPCBA2021(std::string template_name, double speed_ratio);
    bool pcba2021StateInspection();
    void testSucker();

    void getSucherPos(std::string path_name);
    void moveSuckerPos(std::string path_name);
    // TODO: make the copy constructor and the copy assignment operator unavailable
    
    /**
     * @brief Get the Spark Dir object
     * The returned string will be ~/Documents/spark with no forward slash ending.
     * 
     * @return std::string 
     */
    std::string getSparkDir() {
        const char* home = getenv("HOME");
        std::string home_dir(home);
        home_dir += "/Documents/spark";
        return home_dir;
    }; 
    /**
     * @brief urgent test will contain whatever needs to be tested for today
     * 
     */
    void urgentTest(std::string template_name);

    enum class WorkType {
        PICK_FROM_BELT_LINE = 0,
        EMPTY_RIG_ONE,
        EMPTY_RIG_TWO,
        FILL_RIG_ONE,
        FILL_RIG_TWO,
        NO_WORK
    };
    enum class PCBTestResult{
        PASS = 0,
        FAIL
    };
    struct WorkQueueMtxData {
        std::queue<WorkType> work_queue;
        std::mutex mtx;
    };
    WorkQueueMtxData work_queue;
    // TODO: have a thread that monitors the essential pin status and fill the task queue
    void monitorWorkStatus();
    void fillWorkQueue(WorkType work_type);
    void auboMsgCallback();
    void auboMonitorCallback();
    WorkType getTopQueueWork();
    void deleteTopQueueWork();
    std::atomic_bool rig1_empty {false}, 
                     rig1_in_queue_for_work {false},
                     rig1_in_queue_for_pickup {false}, 
                     rig2_empty {false}, 
                     rig2_in_queue_for_work {false},
                     rig2_in_queue_for_pickup {false},
                     board_arrive_on_beltline {false},
                     board_picked {true};
                     
    long double PCB_beltline_counter = 0;
    PCBTestResult rig1_result, rig2_result;
    // construct the ip and connection
    const std::string PLC_IP = "192.168.250.1";
    const uint PLC_PORT = 9600;
    const unsigned PCB_FLIPPER_DM = 3000; // store address of dm used to flip board
    OmronPLC* m_omron_plc_ptr;
    pthread_mutex_t m_mutex_plc;
    
public:
    /**
     * @brief Construct a new Service Manager object
     * Other than the constructor and the destructor, the 
     * @param name 
     */
    ServiceManager(std::string name, int robot_num = 1) :
        m_as(m_nh, name, false),
        m_action_name(name) {
            m_as.registerGoalCallback(boost::bind(&ServiceManager::goalCB, this));
            m_as.registerPreemptCallback(boost::bind(&ServiceManager::preemptCB, this));
            // create the aubo message subscriber thread here
        
            // TODO: wait for the ROBOT server to start and connect to the robot server
            m_robot_ac_ptr = new actionlib::SimpleActionClient<spark_backend::RobotServiceAction>("robot", true);
            ROS_INFO("Waiting for the robot server to start...");
            
            // TODO: uncomment when actually connected to robots
            m_robot_ac_ptr->waitForServer();
            ROS_INFO("Successfully connected to the robot server...");

            // TODO: wait for the VISION server to start and connect to the vision server
            m_vision_ac_ptr = new actionlib::SimpleActionClient<spark_backend::VisionServiceAction>("vision", true);
            ROS_INFO("Waiting for the vision server to start...");

            // TODO: uncomment when actually connected to vision service
            m_vision_ac_ptr->waitForServer();
            ROS_INFO("Successfully connected to the vision server...");

            m_omron_plc_ptr = new OmronPLC(PLC_PORT, PLC_IP);
            // connect to the omron plc
            // TODO: initialize the global parameters
            this->m_nh.setParam(ROS_USER_ROBOT_USAGE_STATUS, ROS_PARAM_USER_STOPPED);

            m_as.start();
            ROS_INFO("Service manager started successfully!");
        }
    
    /**
     * @brief Destroy the Service Manager object
     * This is the service destructor
     * 
     */
    ~ServiceManager(void) {
        // release the resources
        this->m_nh.shutdown();
        this->m_as.shutdown();
        // reset all pointers that are created during initialization
        // destroy the resources that we need
        delete m_robot_ac_ptr;
        delete m_vision_ac_ptr;
        delete m_omron_plc_ptr;
    }


protected:
    // TODO:
};
}

#endif
