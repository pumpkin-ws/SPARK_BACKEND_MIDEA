#include "aubo_move.hpp"

#include <iostream>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <printf.h>

using aubo = spark_robot::AuboMover;

// constructor
aubo::AuboMover(){
    // init variable
    m_robot_state = aubo_robot_namespace::InterfaceCallSuccCode;
    m_thread_run = nullptr;
    m_thread_stop = nullptr;
    m_thread_pause = nullptr;
    m_thread_resume = nullptr;
    m_thread_joint = nullptr;
    m_thread_waypoints = nullptr;

    m_stop_flag = false;
    m_pause_flag = false;
    m_thread_state = STOPPED;
    m_thread_state_joint = STOPPED;
    m_thread_state_waypoints = STOPPED;

    // init aubo
    auboInit();
}

// Add by Hudi 2021.05.21
aubo::AuboMover(std::string server_host, int server_port) {
    // init variable
    SERVER_HOST = server_host.c_str();
    SERVER_PORT = server_port;

    m_robot_state = aubo_robot_namespace::InterfaceCallSuccCode;
    m_thread_run = nullptr;
    m_thread_stop = nullptr;
    m_thread_pause = nullptr;
    m_thread_resume = nullptr;
    m_thread_joint = nullptr;
    m_thread_waypoints = nullptr;

    m_stop_flag = false;
    m_pause_flag = false;
    m_thread_state = STOPPED;
    m_thread_state_joint = STOPPED;
    m_thread_state_waypoints = STOPPED;

    // init aubo
    auboInit();
}

// destructor
aubo::~AuboMover(){
    if(m_thread_run != nullptr){
        delete m_thread_run;
        m_thread_run = nullptr;
    }
    if(m_thread_stop != nullptr){
        delete m_thread_stop;
        m_thread_stop = nullptr;
    }
    if(m_thread_joint != nullptr){
        delete m_thread_joint;
        m_thread_joint = nullptr;
    }
    if(m_thread_waypoints != nullptr) {
        stopWayPointStreaming();
    }
}

int aubo::auboInit(){
    // Interface call: login
    m_robot_state = m_robot_service.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if (m_robot_state == aubo_robot_namespace::InterfaceCallSuccCode) {
        printf("Login successful!\n");
    }
    else {
        printf("Login failed!\n");
    }

    // Tool dynamics parameter
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam)); // why the memory need to be set
    toolDynamicsParam.payload = 4.5;
    toolDynamicsParam.positionX = 0;
    toolDynamicsParam.positionY = 0;
    toolDynamicsParam.positionZ = 0.15;
        
    // If the real robot arm is connected, the arm needs to be initialized.
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    m_robot_state = m_robot_service.rootServiceRobotStartup(toolDynamicsParam/**Tool dynamics parameter**/,
                                               0        /*Collision level*/,
                                               true     /*Whether to allow reading poses defaults to true*/,
                                               true,    /*Leave the default to true */
                                               1000,    /*Leave the default to 1000 */
                                               result); /*Robot arm initialization*/
    
    // check initialization result
    if (m_robot_state == aubo_robot_namespace::InterfaceCallSuccCode) {
        printf("Robot arm initialization succeeded!\n");
    }
    else {
        printf("Robot arm initialization failed!\n");
    }

    // Interface call: Initialize motion properties
    m_robot_service.robotServiceInitGlobalMoveProfile();

    // configurate max joint velocity and acceleration, unit: degree
    double maxJointAcc[aubo_robot_namespace::ARM_DOF] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    double maxJointVel[aubo_robot_namespace::ARM_DOF] = {15.0, 15.0, 15.0, 15.0, 15.0, 15.0};
    aubo::setJointMaxValue(maxJointAcc, maxJointVel);

    return m_robot_state;
}

int aubo::setJointMaxValue(double *vec_acc, double *vec_vel){
    // Interface call: Set the maximum acceleration of the articulated motion
    // The interface requires the unit to be radians
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = vec_acc[0]/180.0*M_PI;
    jointMaxAcc.jointPara[1] = vec_acc[1]/180.0*M_PI;
    jointMaxAcc.jointPara[2] = vec_acc[2]/180.0*M_PI;
    jointMaxAcc.jointPara[3] = vec_acc[3]/180.0*M_PI;
    jointMaxAcc.jointPara[4] = vec_acc[4]/180.0*M_PI;
    jointMaxAcc.jointPara[5] = vec_acc[5]/180.0*M_PI;   
    
    m_robot_service.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    // Interface call: set the maximum speed of articulated motion
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = vec_vel[0]/180.0*M_PI;
    jointMaxVelc.jointPara[1] = vec_vel[1]/180.0*M_PI;
    jointMaxVelc.jointPara[2] = vec_vel[2]/180.0*M_PI;
    jointMaxVelc.jointPara[3] = vec_vel[3]/180.0*M_PI;
    jointMaxVelc.jointPara[4] = vec_vel[4]/180.0*M_PI;
    jointMaxVelc.jointPara[5] = vec_vel[5]/180.0*M_PI;
    m_robot_service.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    return aubo_robot_namespace::InterfaceCallSuccCode;
}

int aubo::auboShutdown(){
    // wait untill all threads end
    if (m_thread_run != nullptr) {
        if (m_thread_run->joinable()) {
            m_thread_run->join();
        }
    }
    if (m_thread_stop != nullptr) {
        if (m_thread_stop->joinable()) {
            m_thread_stop->join();
        }
    }

    // Robotic arm shutdown
    m_robot_service.robotServiceRobotShutdown();
    printf("Robot service shut down!\n");

    // Interface call: logout
    m_robot_service.robotServiceLogout();
    printf("Robot service logout.\n");

    return aubo_robot_namespace::InterfaceCallSuccCode;
}

// essential movement function
void aubo::auboStop(){
    printf("Movement stopped.\n");
    m_robot_state = m_robot_service.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
}

void aubo::auboPause(){
    printf("Movement paused.\n");
    m_robot_state = m_robot_service.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMovePause);
}

void aubo::auboResume(){
    printf("Resume movement.\n");
    m_robot_state = m_robot_service.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveContinue);
}

// func to create start run thread
void aubo::start(){
    m_thread_run = nullptr;
    m_thread_state = STOPPED;
    if (m_thread_run == nullptr && m_thread_state == STOPPED) {
        m_pause_flag = false;
        m_stop_flag = false;
        m_thread_state = RUNNING;
        // m_thread_run = new std::thread(&AuboMover::run, this);
        this->run();
        // m_thread_run->join(); // wait for the thread to join here
    } else {
        printf("The thread state is not STOPPED, or there is a thread running already!\n");
    }
    return;
}

// func to stop thread
void aubo::stop(){
    if (m_thread_run != nullptr && 
       (m_thread_state == RUNNING || m_thread_state == PAUSED)) {
        // change state
        m_pause_flag = false;
        m_stop_flag = true;
        m_condition.notify_all(); // Notify one waiting thread, if there is one.
        auboStop();
        m_thread_state = STOPPED;

        // wait for the threads really stoped
        if (m_thread_run->joinable()) {
//            m_thread_run->join();
        }
        
        // clear pointer
//        delete m_thread_run;
        m_thread_run = nullptr;
    }
    auboStop();
    // FIXME: for debugging purposes, force the run thread pointer ad the thread state to be stopped 
    m_thread_run = nullptr;
    m_thread_state = STOPPED;
    return;
}

// pause run thread
void aubo::pause(){
    if (m_thread_run != nullptr && m_thread_state == RUNNING) {
        m_pause_flag = true;
        m_stop_flag = false;
        auboPause();
        m_thread_state = PAUSED;
    } else {
        auboPause();
    }
    
    return;
}

// resume the paused thread
void aubo::resume(){
    if (m_thread_run != nullptr && m_thread_state == PAUSED){
        // TODO: What does the notify all condition variable do?
        m_condition.notify_all();
        auboResume();
        m_pause_flag = false;
        m_thread_state = RUNNING;
    } else {
        auboResume();
    }
    return;
}

// set a pause point
void aubo::setPausePoint(){
    if (m_pause_flag == true) {
        // achieve mutex lock with unique lock
        std::unique_lock<std::mutex> locker(m_mutex);
        while (m_pause_flag){
            // blocked here! 
            m_condition.wait(locker); // Unlock _mutex and wait to be notified
        }
        locker.unlock();
    }
    return;
}

// movement execute
void aubo::run(){
    printf("Movement started!\n");
    // moveConfig();
    // move();
    return;
}
// static std::mutex joint_mutex;

std::vector<float> aubo::get_m_joints(){
    // std::cout << "Trying to get joint value\n";
    std::lock_guard<std::mutex> lock(m_joint_values.mtx);
    // printf("reading joint values\n");
    // this->m_joint_values.mtx.lock();
    std::vector<float> joint_values = this->m_joint_values.joint_values;
    int size = m_joint_values.joint_values.size();
    // std::cout << "The number of elements in the joint values is " << size << std::endl;
    if(size == 0){
        printf("joints vector is empty!\n");
        return std::vector<float>();
    }
    // this->m_joint_values.mtx.unlock();
    return joint_values;
}

std::vector<float> aubo::get_m_waypoints() {
    std::lock_guard<std::mutex> lock(m_wayPoint_values.mtx);
    // printf("reading way point values\n");
    // this->m_wayPoint_values.mtx.lock();
    std::vector<float> waypoint_values = this->m_wayPoint_values.wayPoint_values;
    int size = m_wayPoint_values.wayPoint_values.size();
    // std::cout << "The number of elements in the way point values is " << size << std::endl;
    if(size == 0){
        return std::vector<float>();
    }
    // this->m_wayPoint_values.mtx.unlock();
    return waypoint_values;
}

// callback function to get joint status
void jointsStatusCallback(const aubo_robot_namespace::JointStatus *jointStatusPtr, 
                                       int size, void* arg){
    std::lock_guard<std::mutex> lock((*reinterpret_cast<spark_robot::AuboMover::JointValues*>(arg)).mtx);
    // printf("writing joints\n");
    ((*reinterpret_cast<spark_robot::AuboMover::JointValues*>(arg)).joint_values).clear();
    for(int i = 0; i < size; i++){
        ((*reinterpret_cast<spark_robot::AuboMover::JointValues*>(arg)).joint_values).push_back(jointStatusPtr[i].jointPosJ);
        // printf("Joint %d: %f \n", i, (*reinterpret_cast<std::vector<float>*>(arg)).back()/M_PI*180);
    }
    // printf("done writing joints\n");
    return; 
}

void aubo::getJointsStatus(){
    if(m_robot_state != aubo_robot_namespace::InterfaceCallSuccCode){
        printf("Robot config wrong, cannot get status!\n");
        return;
    }
    // std::lock_guard<std::mutex> lock(m_joint_values.mtx);
    m_robot_service.robotServiceRegisterRealTimeJointStatusCallback(jointsStatusCallback, &this->m_joint_values);
    return;
}

void wayPointCallback(const aubo_robot_namespace::wayPoint_S *wayPoint, void* arg) {
    std::lock_guard<std::mutex> lock((*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).mtx);
    // printf("writing way points\n");
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.clear();
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->cartPos.position.x);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->cartPos.position.y);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->cartPos.position.z);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->orientation.w);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->orientation.x);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->orientation.y);
    (*reinterpret_cast<spark_robot::AuboMover::WayPointValues*>(arg)).wayPoint_values.push_back(wayPoint->orientation.z);

    // printf("done writing way points\n");
}

void aubo::getWayPointStatus() {
    if(m_robot_state != aubo_robot_namespace::InterfaceCallSuccCode){
        printf("Robot config wrong, cannot get status!\n");
        return;
    }
    std::cout << "getting way point!!" << std::endl;
    // TODO: the lock should not be here
    // std::lock_guard<std::mutex> lock(m_wayPoint_values.mtx);
    m_robot_service.robotServiceRegisterRealTimeRoadPointCallback(wayPointCallback, &this->m_wayPoint_values);
}

std::vector<aubo_robot_namespace::RobotIoDesc> aubo::getAllIOsStatus(IOType type){
    if(m_robot_state != aubo_robot_namespace::InterfaceCallSuccCode){
        printf("Robot config wrong, cannot get status!\n");
        return std::vector<aubo_robot_namespace::RobotIoDesc>();
    }
    if(!m_IOs.empty()){
        m_IOs.clear();
    }
    std::vector<aubo_robot_namespace::RobotIoType> ioTypeVector;
    if(type == DI){
        ioTypeVector.push_back(aubo_robot_namespace::RobotBoardUserDI);
    }
    if(type == DO){
        ioTypeVector.push_back(aubo_robot_namespace::RobotBoardUserDO);
    }
    m_robot_service.robotServiceGetBoardIOConfig(ioTypeVector,m_IOs);
    for(int i = 0; i < m_IOs.size(); i++){
        printf("IO idex: %d \n", i);
        printf("ioName = %s | ", m_IOs[i].ioName);
        printf("ioAddr = %d | ", m_IOs[i].ioAddr);
        printf("ioValue = %f \n", m_IOs[i].ioValue);
    }
    return m_IOs;
} 

// joint streaming thread
void aubo::startJointStreaming() {
    if(m_thread_joint == nullptr && m_thread_state_joint == STOPPED){
        printf("Start streaming joints!\n");
        m_thread_state_joint = RUNNING;
        m_robot_service.robotServiceSetRealTimeJointStatusPush(true);
        m_thread_joint = new std::thread(&aubo::getJointsStatus, this); 
    }
    return;
}

void aubo::stopJointStreaming(){
    if(m_thread_joint != nullptr && m_thread_state_joint == RUNNING){
        m_robot_service.robotServiceSetRealTimeJointStatusPush(false);
        m_thread_state_joint = STOPPED;

        // wait for the threads really stoped
        if (m_thread_joint->joinable()) {
            m_thread_joint->join();
        }
        printf("Joint streaming callback terminated!\n");

        // clear pointer
        delete m_thread_joint;
        m_thread_joint = nullptr;
    }
    return;
}

void aubo::startWayPointStreaming() {
    printf("in way point starter\n");
    if (m_thread_waypoints == nullptr && m_thread_state_waypoints == STOPPED) {
        printf("Start way point streaming!\n");
        m_thread_state_waypoints = RUNNING;
        m_robot_service.robotServiceSetRealTimeRoadPointPush(true);
        std::cout << "starting the get way point thread" << std::endl;
        m_thread_waypoints = new std::thread(&aubo::getWayPointStatus, this);
    }
    return;
};

void aubo::stopWayPointStreaming() {
    if(m_thread_waypoints == nullptr && m_thread_state_waypoints == RUNNING) {
        m_robot_service.robotServiceSetRealTimeRoadPointPush(false);
        m_thread_state_waypoints = STOPPED;

        if(m_thread_waypoints->joinable()) {
            m_thread_waypoints->join();
        }

        printf("Way point streaming callback terminated");

        delete m_thread_waypoints;
        m_thread_waypoints = nullptr;
    }

}

int aubo::io2addrLookUpTable(IOType type, int idx) {
    if (idx < MIN_IO_INDEX || idx > MAX_IO_INDEX || idx == 8 || idx == 9) {
        // idx 8 and 9 are not defined in controller
        printf("Invalid IO index!\n");
        return ERR_IO_INDEX;
    }
    int addr = 0;
    // Mapping of DO_00~DO_17(except idex 8 and 9)
    if (type == DO) {
        if (idx < 8) {
            addr = 32 + idx; // number 32 determinated by the aubo supporting book
        }
        else if (idx >9) {
            addr = 32 + idx - 2; // number 2 estimated according to absence of idx 8 and 9
        }
    }
    else if (type == DI) {
        if (idx < 8) {
            addr = 36 + idx; // number 36 determinated by the aubo supporting book
        }
        else if (idx >9) {
            addr = 36 + idx - 2; // number 2 estimated according to absence of idx 8 and 9
        }
    }
    else if (type == F) {
        addr = 29 + idx; // number 29 determinated by the aubo supporting text book
    }
    return addr;
}

// TODO: define return error value
float aubo::getSingleIOStatus(IOType type, int idx) {
    if (m_robot_state != aubo_robot_namespace::InterfaceCallSuccCode) {
        printf("Robot config wrong, cannot get status!\n");
        return -1.0;
    }
    if (idx > MAX_IO_INDEX || idx < MIN_IO_INDEX || idx == NON_EXISTENT_INDEX1 || idx == NON_EXISTENT_INDEX2) {
        printf("Index %d is invalid IO index!\n", idx);
        return -1.0; // 0 is 
    }
    double value = 0.0;
    aubo_robot_namespace::RobotIoType ioType;
    if (type == DI || type == F) {
        ioType = aubo_robot_namespace::RobotBoardUserDI;
    }
    if (type == DO) {
        ioType = aubo_robot_namespace::RobotBoardUserDO;
    }
    int ioAddr = io2addrLookUpTable(type, idx);
    if (ioAddr == ERR_IO_INDEX) {
        return static_cast<float>(-1.0);
    }
    this->m_robot_service.robotServiceGetBoardIOStatus(ioType, ioAddr, value);
    // printf("idx: %d, value: %f\n", idx, value);
    return static_cast<float>(value);
}

float aubo::setSingleIOStatus(IOType type, int idx, float value) {
    if (m_robot_state != aubo_robot_namespace::InterfaceCallSuccCode) {
        printf("Robot config wrong, cannot get status!\n");
        return 0.0;
    }
    if (idx > MAX_IO_INDEX || idx < MIN_IO_INDEX) {
        printf("Invalid IO index! : %d\n", idx);
        return 0.0;
    }
    aubo_robot_namespace::RobotIoType ioType = aubo_robot_namespace::RobotBoardUserDO;
    int ioAddr = io2addrLookUpTable(type, idx);
    if (ioAddr == ERR_IO_INDEX) {
        return static_cast<float>(-1.0);
    }
    m_robot_state = m_robot_service.robotServiceSetBoardIOStatus(ioType, ioAddr, value);
    printf("idx: %d, value: %f\n", idx, value);
    return 1.0;
}

void aubo::getRobotStatus(RobotState& robot_state) {
    aubo_robot_namespace::RobotState rs;
    this->m_robot_service.robotServiceGetRobotCurrentState(rs);
    robot_state = static_cast<RobotState>(static_cast<int>(rs));
}

std::map<int, int> aubo::getAllDIValues(){
    std::vector<aubo_robot_namespace::RobotIoDesc> all_DIs = this->getAllIOsStatus(IOType::DI);
    std::map<int, int> return_result;
    for(auto elem : all_DIs) {
        return_result.insert(std::pair<int, int>((int)elem.ioAddr, (int)elem.ioValue));
    }
    return return_result;
};

std::map<int, int> aubo::getAllDOValues() {
    std::vector<aubo_robot_namespace::RobotIoDesc> all_DOs = this->getAllIOsStatus(IOType::DO);
    std::map<int, int> return_result;
    for(auto elem : all_DOs) {
        return_result.insert(std::pair<int, int>((int)elem.ioAddr, (int)elem.ioValue));
    }
    return return_result;
}
