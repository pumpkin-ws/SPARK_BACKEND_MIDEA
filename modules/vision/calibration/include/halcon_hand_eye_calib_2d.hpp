/**
 * @file halcon_cam_calib_2d.hpp
 * @author WYJ (you@domain.com)
 * @brief Calaulate the 2d affine transformation of the 
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef HALCON_HAND_EYE_CALIB_2D_HPP_
#define HALCON_HAND_EYE_CALIB_2D_HPP_

#include "opencv2/opencv.hpp"
#include "HalconCpp.h"
#include "eigen3/Eigen/Core"
#include "vision/utilities/include/halcon_utils.hpp"
#include <unistd.h>
#include "common.hpp"


namespace spark_vision {
namespace hand_eye_calib_2d {

/**
 * @brief This procedure calculates the hand-eye calibration matrix 
 *        by means of affine transformation. 
 * @param input_point_coordinate txt file,point coordinates
 * @param input_robot_coordinate txt file,robot tool coordinates
 * @param hand_eye_matrix eigen matrix3d
 * @param output_path hand eye matrix output path
 */
void halconHandEyeAffine(
    cv::Mat image, 
    double threshold_value,
    std::string input_robot_coordinate, 
    Eigen::Matrix3d &hand_eye_matrix,
    std::string output_path);

/**
 * @brief This procedure calculates the hand-eye calibration matrix 
 *        by means of transmission transformation.
 * @param point_coordinate_file input file of point coordinate
 * @param robot_coordinate_file input file of robot corrdinate
 * @param hand_eye_matrix 
 * @param output_path 
 */
void halconHandEyeTransmission(
    cv::Mat image, 
    double threshold_value,
    std::string robot_coordinate_file, 
    Eigen::Matrix3d & cam_in_tool,
    std::string output_path);
    
/**
 * @brief This procedure reads the hand-eye matrix from txt file.
 * 
 * @param input_path txt file,hand eye matrix path
 * @param cam_in_tool Eigen::Matrix3d,hand_eye_matrix  
 */
void readHandEyeMatrix(
    std::string input_path,
    Eigen::Matrix3d & cam_in_tool);

/**
 * @brief This procedure calculates eye in hand robot pose using eye_in_hand matrix.
 * 
 * @param points_coordinate vector<cv::Point2d>，The pixel coordinates of a group of points 
 *                          obtained from the same camera position 
 * @param initial_pos Point2d ,robot initial photo position
 * @param acquisition_pos Point2d ,robot current photo position
 * @param hand_eye_matrix Matrix3d
 * @param robot_pose Tool coordinates of points
 */
void halconCalcMovingRobotPose(
    std::vector<cv::Point2d> points_coordinate,
    cv::Point2d initial_pos,
    cv::Point2d acquisition_pos,
    Eigen::Matrix3d hand_eye_matrix,
    std::vector<cv::Point2d> &robot_pose);
    int readEyeInHandMatrix(std::string pathname, Eigen::Matrix4f& matrix_out);
}
}

#endif