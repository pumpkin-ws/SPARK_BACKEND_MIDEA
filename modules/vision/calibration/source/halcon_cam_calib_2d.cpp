#include "halcon_cam_calib_2d.hpp"
#include <fstream>
void spark_vision::cam_calib_2d::halconCamCalib(std::vector<cv::Mat> images, 
                                  std::string calib_input_path, 
                                  HalconCpp::HTuple& cam_param,
                                  std::string output_path) {

    using namespace HalconCpp;
    //Calibration file and parameters preparation 
    cv::Mat calib_image = images[0];
    int image_rows = calib_image.rows;
    int image_cols = calib_image.cols;
    HTuple hv_TmpCtrl_ReferenceIndex;
    hv_TmpCtrl_ReferenceIndex = 0;
    //string transfer to htuple
    const char*PlateDescriptionChar = calib_input_path.c_str();
    HTuple hv_TmpCtrl_PlateDescription(PlateDescriptionChar);
    HTuple hv_StartParameters;
    hv_StartParameters.Clear();
    // FIXME: undefined, uncommented number literals!! 
    hv_StartParameters[0] = "area_scan_polynomial";
    hv_StartParameters[1] = 0.008;
    hv_StartParameters[2] = 0;
    hv_StartParameters[3] = 0;
    hv_StartParameters[4] = 0;
    hv_StartParameters[5] = 0;
    hv_StartParameters[6] = 0;
    hv_StartParameters[7] = 8.3e-06;
    hv_StartParameters[8] = 8.3e-06;
    hv_StartParameters[9] = image_cols/2;
    hv_StartParameters[10] = image_rows/2;
    hv_StartParameters[11] = image_cols;
    hv_StartParameters[12] = image_rows;
    HTuple hv_TmpCtrl_FindCalObjParNames;
    hv_TmpCtrl_FindCalObjParNames.Clear();
    hv_TmpCtrl_FindCalObjParNames[0] = "gap_tolerance";
    hv_TmpCtrl_FindCalObjParNames[1] = "alpha";
    hv_TmpCtrl_FindCalObjParNames[2] = "skip_find_caltab";
    HTuple hv_TmpCtrl_FindCalObjParValues;
    hv_TmpCtrl_FindCalObjParValues.Clear();
    hv_TmpCtrl_FindCalObjParValues[0] = 1;
    hv_TmpCtrl_FindCalObjParValues[1] = 1;
    hv_TmpCtrl_FindCalObjParValues[2] = "false";
    //calibration
    HTuple hv_CalibHandle;
    CreateCalibData("calibration_object", 1, 1, &hv_CalibHandle);
    SetCalibDataCamParam(hv_CalibHandle, 0, HTuple(), hv_StartParameters);
    SetCalibDataCalibObject(hv_CalibHandle, 0, hv_TmpCtrl_PlateDescription);
    int image_num = images.size();
    for (int index = 0; index < image_num; ++index){
        cv::Mat calib_image = images[index];
        HalconCpp::HObject image = spark_vision::cvMatToHobject(calib_image);
        HObject ho_GrayImage;
        Rgb1ToGray(image, &ho_GrayImage);
        FindCalibObject(ho_GrayImage, hv_CalibHandle, 0, 0, index, hv_TmpCtrl_FindCalObjParNames, 
                        hv_TmpCtrl_FindCalObjParValues);
    }
    HTuple hv_TmpCtrl_Errors;
    CalibrateCameras(hv_CalibHandle, &hv_TmpCtrl_Errors);
    GetCalibData(hv_CalibHandle, "camera", 0, "params", &cam_param);
    WriteCamPar(cam_param, output_path.data());
};


void spark_vision::cam_calib_2d::halconCamParamToEigen(const std::string input_path,
                                         std::string internal_param_output_path, 
                                         std::string distortion_output_path,
                                         Eigen::Matrix3d & internal_param, 
                                         std::vector<double> & distortion) {

    using namespace HalconCpp;
    HTuple hv_camera_param;
    ReadCamPar(input_path.data(), &hv_camera_param);
    //trans cam_param(internal parameter and distortion coefficients) to internal parameter matrix
    HTuple hv_cam_matrix;
    hv_cam_matrix[0] = hv_camera_param[1]/hv_camera_param[7];
    hv_cam_matrix[1] = 0;
    hv_cam_matrix[2] = hv_camera_param[9];
    hv_cam_matrix[3] = 0;
    hv_cam_matrix[4] = hv_camera_param[1]/hv_camera_param[8];
    hv_cam_matrix[5] = hv_camera_param[10];
    hv_cam_matrix[6] = 0;
    hv_cam_matrix[7] = 0;
    hv_cam_matrix[8] = 1;
    Eigen::MatrixXd cam_matrix;
    spark_vision::htupleToEigenMatrix(hv_cam_matrix,cam_matrix);
    internal_param = cam_matrix;
    // write internal matrix to txt
    std::ofstream out(internal_param_output_path.c_str());
    for (unsigned int i = 0; i < internal_param.rows(); ++i) {
        for (unsigned int j = 0; j < internal_param.cols(); ++j) {
            out << internal_param(i, j);
            if (j != (internal_param.cols() - 1)) {
                out << ",";
            }
        }
        out << std::endl;
    }
    //get distortion coefficients
    HTuple hv_cam_k1;
    hv_cam_k1 = ((const HTuple&)hv_camera_param)[2];
    double cam_k1= hv_cam_k1.D();
    distortion.push_back(cam_k1);
    
    HTuple hv_cam_k2;
    hv_cam_k2 = ((const HTuple&)hv_camera_param)[3];
    double cam_k2 = hv_cam_k2.D();
    distortion.push_back(cam_k2);

    HTuple hv_cam_p1;
    hv_cam_p1 = ((const HTuple&)hv_camera_param)[5];
    double cam_p1 = hv_cam_p1.D();
    distortion.push_back(cam_p1);

    HTuple hv_cam_p2;
    hv_cam_p2 = ((const HTuple&)hv_camera_param)[6];
    double cam_p2 = hv_cam_p2.D();
    distortion.push_back(cam_p2);

    HTuple hv_cam_k3;
    hv_cam_k3 = ((const HTuple&)hv_camera_param)[4];
    double cam_k3 = hv_cam_k3.D();
    distortion.push_back(cam_k3);
    //write distortion to txt
    std::ofstream distortion_out(distortion_output_path.c_str());
    for (unsigned int i = 0; i < 5; ++i) {
            distortion_out << distortion[i];
            if (i != 4) {
                distortion_out << ",";
            }
        }
    distortion_out << std::endl;
}

void spark_vision::cam_calib_2d::readCamParam(std::string internal_param_input_path,
                                              std::string distortion_input_path,
                                              Eigen::Matrix3d &internal_param,
                                              std::vector<double> & distortion) {
    //read internal_param from txt to Matrix3d
    std::ifstream in(internal_param_input_path.c_str());
    char buffer[1024] = {'\0'};
    int rows = internal_param.rows();
    int cols = internal_param.cols();
    int index_row = 0;
    while (in.getline(buffer, 1024)) {
        if (index_row >= rows) {
            LOG_ERROR("The number of rows exceeds the limit");
        }
        int index_col = 0;
        sscanf(buffer, "%lf,%lf,%lf", &internal_param(index_row, index_col),
            &internal_param(index_row, index_col+1), &internal_param(index_row, index_col+2));
        if (index_col >= cols) {
            LOG_ERROR("The number of columns exceeds the limit");
        }
        index_row++;
    }
    //read distortion form txt to vector
    std::ifstream distortion_in(distortion_input_path.c_str());
    char distortion_buffer[1024] = {'\0'};
    while (distortion_in.getline(distortion_buffer, 1024)) {
        sscanf(distortion_buffer, "%lf,%lf,%lf,%lf,%lf", &distortion[0],&distortion[1], &distortion[2],
               &distortion[3], &distortion[4]);
    }
};

void spark_vision::cam_calib_2d::writeCamParam(std::string internal_param_output_path, 
                                               std::string distortion_output_path,
                                               Eigen::Matrix3d internal_param,
                                               std::vector<double> distortion) {

};

void spark_vision::cam_calib_2d::halconDistortionCorrection(Eigen::Matrix3d internal_param, 
                                              std::vector<double> distortion, 
                                              cv::Mat& image) {
//Get camera internal parameters （HTuple）
HalconCpp::HTuple cam_param;
double sx = 8.3*1e-06;
double sy = 8.3*1e-06;
cam_param[0] = "area_scan_polynomial";
cam_param[1] = internal_param(2,2)*sy;//f
cam_param[2] = distortion[0];//k1
cam_param[3] = distortion[1];//k2
cam_param[4] = distortion[4];//k3
cam_param[5] = distortion[2];//p1
cam_param[6] = distortion[3];//p2
cam_param[7] = sx;   //sx
cam_param[8] = sy;   //sy
cam_param[9] = internal_param(0,2);    //cx
cam_param[10] = internal_param(1,2);   //cy
int image_rows = image.rows;
int image_cols = image.cols;
cam_param[11] = image_cols;   //halcon row
cam_param[12] = image_rows;   //halcon column
HalconCpp::HTuple camera_param_output;
HalconCpp::ChangeRadialDistortionCamPar("adaptive", cam_param,
        ((((HalconCpp::HTuple(0.0).Append(0.0)).Append(0.0)).Append(0.0)).Append(0.0)), &camera_param_output);
HalconCpp::HObject image_input = spark_vision::cvMatToHobject(image);
HalconCpp::HObject image_output;
HalconCpp::ChangeRadialDistortionImage(image_input, image_input, 
        &image_output, cam_param, camera_param_output);
        HalconCpp::HTuple window;
//spark_vision::displayHalconImage(image_output,window);
// HalconCpp::WriteImage(image_output, "png", 0, 
//                          "../build/distortion_corrected_image.png");
//转halcon到opencv
image = spark_vision::hobjectToCvMat(image_output);
};

void spark_vision::cam_calib_2d::distortionCorrection(Eigen::Matrix3d internal_param, 
                                        std::vector<double> distortion,
                                        cv::Mat& image) {

};
