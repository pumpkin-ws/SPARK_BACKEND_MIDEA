#ifndef HALCON_UTILS_HPP_
#define HALCON_UTILS_HPP_

#include <opencv2/opencv.hpp>
#include <HalconCpp.h>
#include <HalconCDefs.h>
#include <HProto.h>
#include "eigen3/Eigen/Core"

namespace spark_vision {
    /**
     * @brief Convert an cv::Mat image to hobject image
     * 
     * @param image : the input cv::Mat image, can either be a three channel color or an one channel gray.
     * @return HalconCpp::HObject : the return HObject image
     */
    HalconCpp::HObject cvMatToHobject(cv::Mat& image);

    /**
     * @brief Convert an hobject image to an cv::Mat image
     * 
     * @param hobject_image : the input hobject image
     * @return cv::Mat : the return cv::Mat image
     */
    cv::Mat hobjectToCvMat(HalconCpp::HObject hobject_image);

    /**
     * @brief Diplay halcon image, this function is used for creating a window and
     * checking the image. The window should be destroyed after viewing
     * 
     * @param image the halcon image as an hobject
     */
    void displayHalconImage(HalconCpp::HObject image, HalconCpp::HTuple &h_window_handle);    

    /**
     * @brief This procedure opens a new graphics window and adjusts the size
     * such that it fits into the limits specified by WidthLimit
     * and HeightLimit, but also maintains the correct image aspect ratio.
     * 
     * This is just a helper function for debugging purposes, do not spend too much time here testing
     * Some variables are direct copies from halcon, so the naming conventions may be off
     * 
     * @param ho_Image : the image to be displayed
     * @param hv_Row : the number of rows of the image
     * @param hv_Column : the number of columns of the image
     * @param hv_WidthLimit : the width limit of the window
     * @param hv_HeightLimit : the height limit of the window
     * @param hv_WindowHandle : the window handle
     */
    void deviceOpenWindowFitImage (HalconCpp::HObject ho_Image,      HalconCpp::HTuple hv_Row, 
                                   HalconCpp::HTuple hv_Column,      HalconCpp::HTuple hv_WidthLimit, 
                                   HalconCpp::HTuple hv_HeightLimit, HalconCpp::HTuple *hv_WindowHandle);
                                   
    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Convert a halcon tuple to the eigen matrix. This passed in htuple needs to be a squared number.
     * 
     * @param hv_Matrix input halcon matrix
     * @param eigen_matrix output eigen matrix
     */
    void htupleToEigenMatrix(HalconCpp::HTuple hv_tuple, Eigen::MatrixXd &eigen_matrix);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Convert an eigen matrix to halcon tuple.
     * 
     * @param eigen_matrix input eigen matrix
     * @param hv_tuple output halcon matrix
     */
    void eigenMatrixToHtuple(Eigen::MatrixXd eigen_matrix, HalconCpp::HTuple &hv_tuple);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Convert a halcon tuple to std vector.
     * 
     * @param hv_Tuple intput halcon array
     * @param element_number input array element number
     * @param vec output std vector
     */
    void htupleToVector(HalconCpp::HTuple hv_tuple, std::vector <double> &vec);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Convert a std vector to halcon tuple.
     * 
     * @param vec input std vector
     * @param hv_tuple output halcon array
     */
    void vectorToHtuple(std::vector <double> vec,HalconCpp::HTuple &hv_tuple);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Convert halcon row and column to cvPoint2d
     * 
     * @param point_row input htuple row
     * @param point_column input htuple column
     * @param point_coordinate output cvPoint2d
     * Note that the xy of opencv is opposite to halcon, which has been converted here 
     */
    void htupleToCvPoint2d(
        HalconCpp::HTuple point_row,
        HalconCpp::HTuple point_column,
        cv::Point2d & point_coordinate);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief  Convert cvPoint2d to halcon row and column
     * 
     * @param point_coordinate input cvPoint2d
     * @param point_row  output htuple row
     * @param point_column output htuple column
     * Note that the xy of opencv is opposite to halcon, which has been converted here 
     */
    void cvPoint2dtoHtuple(
        cv::Point2d point_coordinate,
        HalconCpp::HTuple & point_row,
        HalconCpp::HTuple & point_column);

    /**
     * @author  WuYujin (wuyj2@jihualab.com)
     * @brief Calculate the ROI circle center in calibration board.
     * 
     * @param image input calibration image with black-white-cross circle
     * @param points_num input int number,number of points used for calibration
     *                   (2d-hand-eye calibration,projection calibration)
     * @param threshold_value threshold used to filter out the mark,the default value is 80 
     * @param points_coordinate output vector, cv-point2d,
     * Note that the xy of opencv is opposite to halcon, which has been converted here 
     */
    void halconPickPointsForCalib(
        cv::Mat image,
        int points_num,
        double threshold_value,
        std::vector<cv::Point2d> & points_coordinate);

    /**
     * @brief copy from adjust_pose,author sms
     * 
     * @param pathname 
     * @param res 
     */
    void txtToVector(const std::string& pathname, std::vector<std::vector<double> >& res);
}

 #endif