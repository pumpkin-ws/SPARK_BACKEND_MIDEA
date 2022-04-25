/**
 * @file common_midea_wifi_signal_chart.hpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief This chart is intended to record the DI/DO signal numbering on the Aubo controller. 
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**
 * @brief This header file defines the signals tables for the gripper and the inspection table
    The gripper will have 4 tool center points. 
 * 
 */
#ifndef COMMON_TEMP_HPP_
#define COMMON_TEMP_HPP_

// PCB Midea PCB gripper Signals
const unsigned int GRIPPER_DO = 5; 

const unsigned int RIG1_UP_DO = 1;
const unsigned int RIG1_DOWN_DO = 2;
const unsigned int RIG1_PASS_DI = 6;
const unsigned int RIG1_FAIL_DI = 7;
const unsigned int RIG1_UP_DI = 2;
const unsigned int RIG1_DOWN_DI = 3;

const unsigned int RIG2_UP_DO = 3;
const unsigned int RIG2_DOWN_DO = 4;
const unsigned int RIG2_PASS_DI = 10;
const unsigned int RIG2_FAIL_DI = 11;
const unsigned int RIG2_UP_DI = 4;
const unsigned int RIG2_DOWN_DI = 5;

const unsigned int BELTLINE_TRIGGER_DI = 0; 


#endif 