/**
 * @file halcon_cam_calib_2d.hpp
 * @author WYJ (you@domain.com)
 * @brief Calibration of camera intrinsics and distortion coefficients, with halcon used
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef CAM_CALIB_2D_HALCON_HPP_
#define CAM_CALIB_2D_HALCON_HPP_
#include "opencv2/opencv.hpp"
#include "HalconCpp.h"
#include "eigen3/Eigen/Core"
#include "vision/utilities/include/halcon_utils.hpp"
#include "common.hpp"
#include <unistd.h>


namespace spark_vision {
namespace cam_calib_2d {
/**
 * @brief This procedure uses halcon to calibrate the camera internal parameters.
 * 
 * @param images Input the the calibration pictures   
 * @param calib_input_path Input the calibration board description file 
 * @param cam_param Output camera parameters including internal parameter 
 *                  and distortion coefficients[k1,k2,p1,p2,k3].
 * @param output_path Output halcon tuple cam parameter file
 * 
 */
void halconCamCalib(
    std::vector<cv::Mat> images, 
    std::string calib_input_path, 
    HalconCpp::HTuple & cam_param,
    std::string output_path); 

/**
 * @brief This procedure converts the halcon camera parameter file 
 *        into the eigen matrix representing the internal parameter 
 *        and the std vector representing the distortion coefficients.
 * @param input_path  halcon camera parameter file
 * @param internal_param_output_path output txt file includes camera internal parameters
 * @param distortion_output_path output txt file includes distortion coefficients [k1,k2,p1,p2,k3]
 * @param internal_param Output eigen matrix represents internal parameter
 * @param distortion Output std vector represents distortion coefficients [k1,k2,p1,p2,k3]
 */
void halconCamParamToEigen(
    const std::string input_path,
    std::string internal_param_output_path, 
    std::string distortion_output_path,
    Eigen::Matrix3d & internal_param, 
    std::vector<double> & distortion);

/**
 * @brief read camera parameters including internal parameters and distortion coefficients
 * 
 * @param read_path input camera parameter file 
 * @param internal_param output eigen matrix internal_param
 * @param distortion output std vector distortion coefficients [k1,k2,p1,p2,k3]
 */
void readCamParam(
    std::string internal_param_input_path,
    std::string distortion_input_path,
    Eigen::Matrix3d &internal_param,
    std::vector<double> & distortion);

/**
 * @brief Write camera parameters including internal parameters and distortion coefficients 
 *        in wirite path.
 * 
 * @param write_path input txt file,the file to be written to the camera parameters  
 * @param internal_param input eigen matrix internal_param
 * @param distortion input std vector distortion coefficients [k1,k2,p1,p2,k3]
 */
void writeCamParam(
    std::string internal_param_output_path, 
    std::string distortion_output_path,
    Eigen::Matrix3d internal_param,
    std::vector<double> distortion);

/**
 * @brief This procedure uses halcon to correct the distortion of the image 
 * 
 * @param internal_param input eigen matrix represents internal parameter
 * @param distortion iutput std vector represents distortion
 * @param image cv::Mat image,input the image to be corrected and return to the corrected image
 */
void halconDistortionCorrection(
    Eigen::Matrix3d internal_param, 
    std::vector<double> distortion, 
    cv::Mat& image);

/**
 * @brief This procedure uses opencv to correct the distortion of the image 
 * 
 * @param internal_param input eigen matrix represents internal parameter
 * @param distortion iutput std vector represents distortion
 * @param image cv::Mat image,input the image to be corrected and return to the corrected image
 */
void distortionCorrection(
    Eigen::Matrix3d internal_param, 
    std::vector<double> distortion, 
    cv::Mat& image);
}
}
#endif