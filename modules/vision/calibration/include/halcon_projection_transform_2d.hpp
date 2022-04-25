#ifndef HALCON_PROJECTION_TRANSFORM_2D_HPP_
#define HALCON_PROJECTION_TRANSFORM_2D_HPP_
#include "opencv2/opencv.hpp"
#include "HalconCpp.h"
#include "eigen3/Eigen/Core"
#include <iostream>
#include <unistd.h>
#include "vision/utilities/include/halcon_utils.hpp"
#include "common.hpp"

namespace spark_vision {
namespace projection_transform_2d {

/**
 * @brief calculate the projection matrix to transform the tilted image 
 * 
 * @param image cv:Mat image, calib image with points can be extracted to correct tilt 
 * @param threshold_value double,threshold used to filter out the mark,the default value is 80, TODO: what mark?
 * @param input_point_path txt file, includes 4 coordinates which extracted points 
 *                         transformed to.
 * @param projection_matrix eigen matrix3d, output projection matrix
 * @param output_path txt file, which projection matrix saved in
 */
void halconProjectionCalib(
    cv::Mat image, 
    double threshold_value,
    std::string input_point_path, 
    Eigen::Matrix3d & projection_matrix,
    std::string output_path);

/**
 * @brief read projection matrix form a txt file
 * 
 * @param input_path txt file,which projection matrix saved in
 * @param projection_matrix eigen matrix3d, output projection matrix
 */
void readProjectionMatrix(std::string input_path,Eigen::Matrix3d & projection_matrix);

/**
 * @brief this procedure uses halcon to correct tilted image by projection transformation
 * 
 * @param projection_matrix eigen matrix3d, input projection matrix
 * @param image cv::Mat image, input the image to be corrected and return to the corrected image
 */
void halconAntiProjection(Eigen::Matrix3d projection_matrix,cv::Mat & image);
}
}



#endif