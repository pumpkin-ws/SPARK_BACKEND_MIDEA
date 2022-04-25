/**
 * @file aubo_move.hpp
 * @author Dezhu Xiong
 * @brief AuboMover can control the movement of the AUBO robot
 * @version 0.1
 * @date 2021-02-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef AUBO_MOVE_H_
#define AUBO_MOVE_H_

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <condition_variable>
#include "common.hpp"

const double MAX_LINE_VEL{6}; // 6.0
const double MAX_LINE_ACC{6}; // 6.0
const double MAX_JOINT_VEL{4}; // 4
const double MAX_JOINT_ACC{32}; // 32

const std::string ROBOT_HOST_IP_MIDEA{"192.168.1.3"};
const std::string ROBOT_HOST_IP_FACTRY{"192.168.1.32"};

namespace spark_robot{
    class AuboMover{
    public:
        enum ThreadState{
            STOPPED,
            RUNNING,
            PAUSED
        };

        enum IOType{
            DI = 0,
            DO,
            AI,
            AO,
            F
        };     

        struct JointValues{
            std::vector<float> joint_values;
            std::mutex mtx;
        };
        JointValues m_joint_values;      

        struct WayPointValues{
            std::vector<float> wayPoint_values;
            std::mutex mtx;
        };
        std::map<int, int> getAllDIValues();
        std::map<int, int> getAllDOValues();

    private:
        const int MAX_IO_INDEX = 17;
        const int MIN_IO_INDEX = -1;
        const int ERR_IO_INDEX = -1;
        const int NON_EXISTENT_INDEX1 = 8;
        const int NON_EXISTENT_INDEX2 = 9;
        // TODO: delete the copy contructors here
        // TODO: change the service host ip here for different aubo robots, but hardcoding is really not a smart way         
        const char* SERVER_HOST = ROBOT_HOST_IP_MIDEA.c_str(); 
        int SERVER_PORT = 8899; 
        int m_robot_state;
        void auboStop();
        void auboPause();
        void auboResume();
        void getJointsStatus();
        void getWayPointStatus();
        
        std::vector<float> m_joints;

        WayPointValues m_wayPoint_values;

        std::vector<aubo_robot_namespace::RobotIoDesc> m_IOs;
        std::thread *m_thread_stop;
        std::thread *m_thread_pause;
        std::thread *m_thread_resume;
        std::thread *m_thread_joint;
        std::thread *m_thread_waypoints;
        std::mutex m_mutex;
        std::condition_variable m_condition;
        std::atomic_bool m_pause_flag;
        std::atomic_bool m_stop_flag;

        ThreadState m_thread_state;
        ThreadState m_thread_state_joint;
        ThreadState m_thread_state_waypoints;
        std::thread *m_thread_run;

        int io2addrLookUpTable(IOType type, int idx);   

    public:
 
        ServiceInterface m_robot_service; // core variable to control robot
        AuboMover();
        AuboMover(std::string server_host, int server_port); // Add by Hudi 2021.05.21
        virtual ~AuboMover();  // Add 'virtual' by Hudi 2021.05.21

        int auboInit();
        int auboShutdown();
        void stop();
        void start();
        void pause();
        void resume();
        void setPausePoint();
        int setJointMaxValue(double *vec_vel, double *vec_acc);
        bool get_is_running();
        std::vector<aubo_robot_namespace::RobotIoDesc> getAllIOsStatus(IOType type);
        std::vector<float> get_m_joints();
        std::vector<float> get_m_waypoints();
        void startJointStreaming();
        void stopJointStreaming();

        void startWayPointStreaming();
        void stopWayPointStreaming();
        float getSingleIOStatus(IOType type, int idx);
        float setSingleIOStatus(IOType type, int idx, float val);
        void getRobotStatus(RobotState&);

    protected:
        virtual void moveConfig(double fraction) = 0;
        virtual void moveLine() = 0;
        virtual void moveJoint() = 0;
        virtual void moveTrackCartesianP() = 0;
        virtual void moveTrackJoint() = 0;
        virtual void moveTrackArcCir() = 0;
        virtual void run();
    };
}

#endif
