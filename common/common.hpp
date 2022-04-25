#ifndef COMMON_H_
#define COMMON_H_

#include <string>
#include <cmath>
#include <chrono> 

#define RS_IMG_WIDTH 1280
#define RS_IMG_HEIGHT 720

/**
 * @brief utility to log error and debug information to console
 * 
 */
#define LOG_ERROR(...) do {\
    printf("<%s %s> %s %s:%d \033[47;31mERROR: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_WARNING(...) do {\
    printf("<%s %s> %s %s:%d \033[1;33mWARNING: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_DEBUG(...) do {\
    printf("<%s %s> %s %s:%d \033[0;34mDEBUG: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

// TODO: change the serial number to valid camera serial numbers when in use
const std::string EYE_IN_HAND_SERIAL{"943222071284"};//043422250968
const std::string EYE_TO_HAND_SERIAL{""};
const int FRAME_RATE{15};
const int FRAME_WIDTH{1280};
const int FRAME_HEIGHT{720};


const std::string ROS_USER_ROBOT_USAGE_STATUS = "USER_ROBOT_STATUS";
const std::string ROS_PARAM_USER_STOPPED = "USER_STOPPED";
const std::string ROS_PARAM_USER_PAUSED = "USER_PAUSED";
const std::string ROS_PARAM_USER_RESUME = "USER_RESUME";
const std::string ROS_PARAM_USER_RUN = "USER_RUN";
const std::string TERMINATE_WORKFLOW = "TERMINATE_WORKFLOW";
const std::string BACKEND_WARNING = "BACKEND_WARNING";
const std::string CAMERA_STATUS = "CAMERA_STATUS";
const std::string VS_RESTART_ATTEMPT = "VS_RESTART_ATTEMPT"; // signal if vision service has attempted to restart the eihstream
const std::string USE_RIG1 = "USE_RIG1";
const std::string USE_RIG2 = "USE_RIG2";


// different move types of the robot, defined in the aubo_move_utils.hpp
enum class MoveType {
    MOVE_L = 0, 
    MOVE_J,
    MOVE_JOINT_ANGLE,
    TRACK_P,
    TRACK_ARC,
    TRACK_JOINT,
    PAUSE, 
    STOP, 
    START,
    RESUME,
    GET_DI,
    GET_DO,
    SET_DO,
    GET_ROBOT_STATE,
    GET_AUBO_MSG,
    DEFAULT
}; 

enum class ManagerTasks {
    MIDEA_PCBA = 0,
    TEST_FRONTEND,
    GET_AUBO_INFO, 
    GENERATE_MIDEA_PCB_2021_TEMPLATE,
    URGENT_TEST,   
    EIH_CALIBRATION,
    MOVE_HOME,
    PICK_UP_BOARD,
    MOVE_TO_RIG1,
    MOVE_TO_RIG2,
    MOVE_TO_SUCCESS,
    MOVE_TO_FAILURE,
    MOVE_FROM_RIG1_TO_HOME,
    MOVE_FROM_RIG2_TO_HOME,
    GENERATE_CLOSEUP_TEMPALTE,
    RESET_DO,
    TOGGLE_GRIPPER,
    RIG1_UP_TOGGLE,
    RIG2_UP_TOGGLE,
    RIG1_DOWN_TOGGLE,
    RIG2_DOWN_TOGGLE,
    DEFAULT
};

enum class RobotState {
    RobotStopped = 0,
    RobotRunning,
    RobotPaused,
    RobotResumed
};

enum class IOType {
    DI = 0,
    DO,
    AI,
    AO,
    F
}; 

enum class VisionTasks {
    CALIB_INTRINSICS = 0,
    CALIB_PROJECTION,
    CALIB_2D_EYE_IN_HAND,
    CALIB_2D_EYE_TO_HAND,
    CALIB_3D_EYE_IN_HAND,
    CALIB_3D_EYE_TO_HAND,
    GENERATE_3D_TEMPLATE,
    TRACK_2D_OBJECT,
    TRACK_3D_OBJECT,
    TRACK_WIFI_2021,
    GET_EIH_FRAME_DATA,
    TRACK_BARCODE,
    TRACK_PCBA_2021,
    SAVE_CURRENT_FRAME,
};

#endif