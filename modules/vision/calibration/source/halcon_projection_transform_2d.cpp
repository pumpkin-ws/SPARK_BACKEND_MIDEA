#include "halcon_projection_transform_2d.hpp"
#include <fstream>
void spark_vision::projection_transform_2d::halconProjectionCalib(cv::Mat image, 
                                         double threshold_value,
                                         std::string input_point_path, 
                                         Eigen::Matrix3d & projection_matrix,
                                         std::string output_path) {
std::vector<cv::Point2d> points_coordinate;
//get the position of the mark point before transformation 
spark_vision::halconPickPointsForCalib(image,4,threshold_value,points_coordinate);
HalconCpp::HTuple pre_trans_row;
HalconCpp::HTuple pre_trans_column;
for (int index=0; index < 4; ++index){
        cv::Point2d point_coordinate = points_coordinate[index];
        HalconCpp::HTuple row;
        HalconCpp::HTuple column;
        spark_vision::cvPoint2dtoHtuple(point_coordinate,row,column);
        pre_trans_row[index] = row;
        pre_trans_column[index] = column;
}
//get the position of the mark after transformation
std::vector<std::vector<double>> trans_to_points;
HalconCpp::HTuple trans_to_row;
HalconCpp::HTuple trans_to_column;
spark_vision::txtToVector(input_point_path,trans_to_points);
for (int index=0; index < 4; ++index) {
    std::vector<double> trans_to_coordinate = trans_to_points[index];
    trans_to_row[index] = trans_to_coordinate[0];
    trans_to_column[index] = trans_to_coordinate[1];
}
//calculate proojection matrix
HalconCpp::HTuple proj_matrix;
HalconCpp::HomVectorToProjHomMat2d(pre_trans_row, pre_trans_column, (((HalconCpp::HTuple(1).Append(1)).Append(1)).Append(1)), 
            trans_to_row, trans_to_column, (((HalconCpp::HTuple(1).Append(1)).Append(1)).Append(1)), 
            "normalized_dlt", &proj_matrix);
// convert htuple proj_matrix to eigen matrixXd
Eigen::MatrixXd proj_m;
spark_vision::htupleToEigenMatrix(proj_matrix,proj_m);
projection_matrix = proj_m;
//write projection to txt file
    std::ofstream out(output_path.c_str());
    for (unsigned int i = 0; i < projection_matrix.rows(); ++i) {
        for (unsigned int j = 0; j < projection_matrix.cols(); ++j) {
            out << projection_matrix(i, j);
            if (j != (projection_matrix.cols() - 1)) {
                out << ",";
            }
        }
        out << std::endl;
    }

};

void spark_vision::projection_transform_2d::readProjectionMatrix(std::string input_path,
                                        Eigen::Matrix3d & projection_matrix) {
    std::ifstream in(input_path.c_str());
    char buffer[1024] = {'\0'};
    int rows = projection_matrix.rows();
    int cols = projection_matrix.cols();
    int index_row = 0;
    while (in.getline(buffer, 1024)) {
        if (index_row >= rows) {
            LOG_ERROR("The number of rows exceeds the limit");
        }
        int index_col = 0;
        sscanf(buffer, "%lf,%lf,%lf", &projection_matrix(index_row, index_col),
            &projection_matrix(index_row, index_col+1), &projection_matrix(index_row, index_col+2));
        if (index_col >= cols) {
            LOG_ERROR("The number of columns exceeds the limit");
        }
        index_row++;
    }
};

void spark_vision::projection_transform_2d::halconAntiProjection(Eigen::Matrix3d projection_matrix,
                                        cv::Mat & image) {

HalconCpp::HObject anti_proj_image = spark_vision::cvMatToHobject(image);
//trans matrix3d to matrixXd
Eigen::MatrixXd proj_m;
proj_m = projection_matrix;
HalconCpp::HTuple proj_matrix;
spark_vision::eigenMatrixToHtuple(proj_m, proj_matrix);
HalconCpp::HObject image_proj_trans;
HalconCpp::ProjectiveTransImage(anti_proj_image, &image_proj_trans, proj_matrix, 
      "bilinear", "false", "false");
// HalconCpp::WriteImage(image_proj_trans, "png", 0, 
                        //  "../build/proj_trans_image.png");
image = spark_vision::hobjectToCvMat(image_proj_trans);
};