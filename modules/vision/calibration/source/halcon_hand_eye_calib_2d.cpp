#include "halcon_hand_eye_calib_2d.hpp"
#include <fstream>

//Note:The input image coordinate system should same to opencv which is opposite to halcon.
//This function has transformed the coordinate system  
void spark_vision::hand_eye_calib_2d::halconHandEyeAffine(cv::Mat image, 
                                                          double threshold_value,
                                                          std::string input_robot_coordinate,
                                                          Eigen::Matrix3d &hand_eye_matrix,
                                                          std::string output_path) {
    //get point_coordinate from image
    std::vector<cv::Point2d> points_coordinate;
    int point_num = 4;
    spark_vision::halconPickPointsForCalib(image,point_num,threshold_value,points_coordinate);
    HalconCpp::HTuple calib_cam_row;
    HalconCpp::HTuple calib_cam_column;
    for (int index=0; index < point_num; ++index){
            cv::Point2d point_coordinate = points_coordinate[index];
            HalconCpp::HTuple row;
            HalconCpp::HTuple column;
            spark_vision::cvPoint2dtoHtuple(point_coordinate,row,column);
            calib_cam_row[index] = row;
            calib_cam_column[index] = column;
    }
    //read robot_coordinate from txt to std vector<point2d>;
    std::vector<cv::Point2d> robot_coordinate;
    std::ifstream robot_in(input_robot_coordinate.c_str());
    char robot_buffer[1024] = {'\0'};
    while (robot_in.getline(robot_buffer, 1024)) {
        cv::Point2d temp_point;
        sscanf(robot_buffer, "%lf,%lf", &temp_point.x, &temp_point.y);
        robot_coordinate.push_back(temp_point);
    }
    HalconCpp::HTuple calib_robot_pose_x;
    HalconCpp::HTuple calib_robot_pose_y;
    for(int i = 0; i < point_num; ++i) {
    cv::Point2d point;
    point = robot_coordinate[i];
    HalconCpp::HTuple hv_calib_robot_pose_x;
    HalconCpp::HTuple hv_calib_robot_pose_y;
    spark_vision::cvPoint2dtoHtuple(point,hv_calib_robot_pose_x,hv_calib_robot_pose_y);
    calib_robot_pose_x[i]=hv_calib_robot_pose_y;
    calib_robot_pose_y[i]=hv_calib_robot_pose_x;
    } 
    //calculate the hand_eye_matrix matrix
    HalconCpp::HTuple hv_cam_in_tool;
    HalconCpp::VectorToHomMat2d(calib_cam_row, calib_cam_column, calib_robot_pose_x, 
                                calib_robot_pose_y, &hv_cam_in_tool);//2*3 matrix

    hv_cam_in_tool[6] = 0;
    hv_cam_in_tool[7] = 0;
    hv_cam_in_tool[8] = 1;
    Eigen::MatrixXd hand_eye_m;
    spark_vision::htupleToEigenMatrix (hv_cam_in_tool,hand_eye_m);
    hand_eye_matrix = hand_eye_m;
    //write hand eye matrix to txt
    std::ofstream out(output_path.c_str());
    for (unsigned int i = 0; i < hand_eye_matrix.rows(); ++i) {
        for (unsigned int j = 0; j < hand_eye_matrix.cols(); ++j) {
            out << hand_eye_matrix(i, j);
            if (j != (hand_eye_matrix.cols() - 1)) {
                out << ",";
            }
        }
        out << std::endl;
    }
};


void spark_vision::hand_eye_calib_2d::halconHandEyeTransmission(cv::Mat image, 
                                                                double threshold_value,
                                                                std::string robot_coordinate_file, 
                                                                Eigen::Matrix3d & cam_in_tool,
                                                                std::string output_path) {
    //get point_coordinate from image
    std::vector<cv::Point2d> points_coordinate;
    int point_num = 4;
    spark_vision::halconPickPointsForCalib(image,point_num,threshold_value,points_coordinate);
    HalconCpp::HTuple calib_cam_row;
    HalconCpp::HTuple calib_cam_column;
    for (int index=0; index < point_num; ++index){
            cv::Point2d point_coordinate = points_coordinate[index];
            HalconCpp::HTuple row;
            HalconCpp::HTuple column;
            spark_vision::cvPoint2dtoHtuple(point_coordinate,row,column);
            calib_cam_row[index] = row;
            calib_cam_column[index] = column;
    }
    //read robot_coordinate from txt to std vector<point2d>;
    std::vector<cv::Point2d> robot_coordinate;
    std::ifstream robot_in(robot_coordinate_file.c_str());
    char robot_buffer[1024] = {'\0'};
    while (robot_in.getline(robot_buffer, 1024)) {
        cv::Point2d temp_point;
        sscanf(robot_buffer, "%lf,%lf", &temp_point.x, &temp_point.y);
        robot_coordinate.push_back(temp_point);
    }
    HalconCpp::HTuple calib_robot_pose_x;
    HalconCpp::HTuple calib_robot_pose_y;
    for(int i = 0; i < point_num; ++i) {
    cv::Point2d point;
    point = robot_coordinate[i];
    HalconCpp::HTuple hv_calib_robot_pose_x;
    HalconCpp::HTuple hv_calib_robot_pose_y;
    spark_vision::cvPoint2dtoHtuple(point,hv_calib_robot_pose_x,hv_calib_robot_pose_y);
    calib_robot_pose_x[i]=hv_calib_robot_pose_y;
    calib_robot_pose_y[i]=hv_calib_robot_pose_x;
    } 
    //calculate the cam_in_tool matrix
    HalconCpp::HTuple hv_HomMat2D;
    HalconCpp::HTuple hv_Covariance;
    HalconCpp::VectorToProjHomMat2d(calib_cam_row, calib_cam_column, calib_robot_pose_x, 
                                    calib_robot_pose_y, "normalized_dlt", HalconCpp::HTuple(), HalconCpp::HTuple(), 
                                    HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), &hv_HomMat2D, &hv_Covariance);//3*3 matrix
    Eigen::MatrixXd hand_eye_matirx;
    spark_vision::htupleToEigenMatrix(hv_HomMat2D,hand_eye_matirx);
    cam_in_tool = hand_eye_matirx;
    //write hand eye matrix to txt
    std::ofstream out(output_path.c_str());
    for (unsigned int i = 0; i < cam_in_tool.rows(); ++i) {
        for (unsigned int j = 0; j < cam_in_tool.cols(); ++j) {
            out << cam_in_tool(i, j);
            if (j != (cam_in_tool.cols() - 1)) {
                out << ",";
            }
        }
        out << std::endl;
    }
};
        
void spark_vision::hand_eye_calib_2d::readHandEyeMatrix(std::string input_path,
                                                        Eigen::Matrix3d & cam_in_tool) {
    std::ifstream in(input_path.c_str());
        char buffer[1024] = {'\0'};
        int rows = cam_in_tool.rows();
        int cols = cam_in_tool.cols();
        int index_row = 0;
        while (in.getline(buffer, 1024)) {
            if (index_row >= rows) {
                LOG_ERROR("The number of rows exceeds the limit");
            }
            int index_col = 0;
            sscanf(buffer, "%lf,%lf,%lf", &cam_in_tool(index_row, index_col),
                &cam_in_tool(index_row, index_col+1), &cam_in_tool(index_row, index_col+2));
            if (index_col >= cols) {
                LOG_ERROR("The number of columns exceeds the limit");
            }
            index_row++;
        }
};

void spark_vision::hand_eye_calib_2d::halconCalcMovingRobotPose(std::vector<cv::Point2d> points_coordinate,
                                                                cv::Point2d initial_pos,
                                                                cv::Point2d acquisition_pos,
                                                                Eigen::Matrix3d hand_eye_matrix,
                                                                std::vector<cv::Point2d> &robot_pose) {
    int point_num = points_coordinate.size();
    for (int index = 0; index < point_num; ++index) {
        cv::Point2d point_coor = points_coordinate[index];
        //坐标xy正反问题？
        double robot_base_x = (hand_eye_matrix(0,0) * point_coor.y + hand_eye_matrix(0,1) * point_coor.x + hand_eye_matrix(0,2))
                               /(hand_eye_matrix(2,0) * point_coor.y + hand_eye_matrix(2,1) * point_coor.x + hand_eye_matrix(2,2))
                               - initial_pos.x + acquisition_pos.x;
        double robot_base_y = (hand_eye_matrix(1,0) * point_coor.y + hand_eye_matrix(1,1) * point_coor.x + hand_eye_matrix(1,2))
                               /(hand_eye_matrix(2,0) * point_coor.y + hand_eye_matrix(2,1) * point_coor.x + hand_eye_matrix(2,2))
                               - initial_pos.y + acquisition_pos.y;
        cv::Point2d robot_pose_coordinate;
        robot_pose_coordinate.x=robot_base_x;
        robot_pose_coordinate.y=robot_base_y;
        robot_pose.push_back(robot_pose_coordinate);
    }
};

int spark_vision::hand_eye_calib_2d::readEyeInHandMatrix(std::string pathname, Eigen::Matrix4f& matrix_out)
{
    std::ifstream infile(pathname);
    std::vector<std::vector<float>> data_all;
    std::vector<float> data_row;
    std::string data_string;
    if (pathname.empty()) {
        LOG_ERROR("no such file");
        return 1;
    }
    
    while (getline(infile, data_string)) {
        char *data_char = (char*)data_string.c_str(); // convert to char format
        const char *split = ",";
        char *data_remain = strtok(data_char,split); 
        float temp;
        while (data_remain != NULL) {
            temp = atof(data_remain);
            data_row.push_back(temp);
            data_remain = strtok(NULL, split);
        }
        data_all.push_back(data_row);
        data_row.clear();
        data_string.clear();
    }

    for (size_t i = 0; i < data_all.size(); i++) {
        if (data_all.size() == matrix_out.rows()) {
            for (size_t j = 0; j < data_all[i].size(); j++) {
                if (data_all[i].size() == matrix_out.cols()) {
                    matrix_out(i,j) = data_all[i][j];
                } else {
                    LOG_ERROR("data cols is not coincident with input matrix,such as \n1,2,3,4\n5,6,7,8\n1,2,3,4\n5,6,7,8");
                    return 1;
                }   
            }           
        } else {
            LOG_ERROR("data rows error is not coincident with input matrix,such as \n1,2,3,4\n5,6,7,8\n1,2,3,4\n5,6,7,8");
            return 1;
        } 
    }
    infile.close();
    return 0; 
}