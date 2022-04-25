/**
 * @file halcon_eye_in_hand_3d.hpp
 * @author Lv Xiaoge (pumpkin_wang@foxmail.com)
 * @brief the parameters of the file(readParametesFromFile):
            1)the number of the calibration images;
            2)the size of the calibration plate;
            3)the focus length of the camera;
            4)the X coordinate value of center point int the images;
            5)the Y coordinate value of center point int the images;
            6)the width of the image;
            7)the hight of the image;
            This class include four parts£º
            1£©readParametesFromFile£»
            2£©calculatePoseFromCalibrationPlate£»
            3)readRobotPoseFromFile;
            4)calibrateAndSaveResult;
            In order to successfully use this class,you must use these four functions from one to four 
            and at last you can get the Eye In Hand calibration result and the calibration error.
            Note(the unit of the robot poses):The unit of the translation matrix is meters;
            The unit of the rotation matrix is degrees.
            Created by XiaoGeLv   created on 20210707
 * @version 0.1
 * @date 2021-08-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef HALCON_EYE_IN_HAND_3D_HPP_
#define HALCON_EYE_IN_HAND_3D_HPP_
#include "HalconCpp.h"
#include "yaml-cpp/yaml.h"
#include <string>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

namespace spark_vision {
class CalibrateEyeInHand3D_Halcon{
public:
    /** readParametesFromFile
     * @brief Read the calibration parametes from file,including
     * 1 - the number of the calibration images
     * 2 - the description file of the calibration plate
     * 3 - the camera internal parameters
     * @return HalconCpp::HTuple CalibrateParameters:the tuple of the parameters
     * Read internal parameters
     */
    void readCalibParametersFromYAML(const std::string& calib_param_YAML);

    /**
     * @brief Read in the camera poses from robot pose yaml files
     * 
     * @param RobotPoseYAML 
     */
    void readRobotPoseFromYAML(const std::string& robot_pose_YAML);

    /**
     * @brief Calculate the poses of all calibration plate images
     * the parameters read off include: 
     * 1 - the description file of the calibration plate (This is needed by halcon)?
     * 2 - the camera internal parameters
     * 
     * The camera parameters will be stored in m_calibrateDataID
     * 
     * @param ImageFilePath 
     */
    void calculateCalibrationPlatePoses(const std::string& image_file_path);
    
    /**
     * @brief perform hand-eye calibration and save the result to the designated result path
     * 
     * @param result_path 
     */
    void calibrateAndSaveResult(const std::string& result_path);
    
    /**
     * @brief read in the calibration result and convert to a 4x4 matrix
     * 
     * @param CalibrateResultPath 
     */
    Eigen::Matrix4d readCalibrateResult(const std::string& CalibrateResultPath);
    Eigen::Matrix4d m_calibrateMatrix = Eigen::Matrix4d::Identity();

private:
    HalconCpp::HTuple m_mumImages; //the number of the calibration images
    HalconCpp::HTuple m_calTabFile; //the description file of the calibration plate
    HalconCpp::HTuple m_cameraInternalParameters; //the camera internal parameters
    HalconCpp::HTuple m_toolInCamPose; //the result of the eye in hand calibration
    HalconCpp::HTuple m_resultErrors; //the errors of the eye in hand calibration
    HalconCpp::HTuple m_calibrateParameters; //include: m_mumImages,m_calTabFile,m_cameraInternalParameters
    /**
     * @brief FIXME: What does this function do??
     * 
     * @param hv_CalibDataID 
     * @param hv_RotationTolerance 
     * @param hv_TranslationTolerance 
     * @param hv_Warnings 
     */
    void checkInputposes(
        HalconCpp::HTuple hv_CalibDataID, 
        HalconCpp::HTuple hv_RotationTolerance,
        HalconCpp::HTuple hv_TranslationTolerance, 
        HalconCpp::HTuple *hv_Warnings);
    
    HalconCpp::HTuple m_toolInBasePose;//the tool coordinate value in the robot base coordinate system
    HalconCpp::HTuple m_camCalibrateID;// the handle of the caculating the model in the camera system
    HalconCpp::HTuple m_calibrateID;//the handle of the entire calibration process
    HalconCpp::HTuple m_calculateObjInCamPose;//calculate the pose of the calibration plate
    };

}

#endif