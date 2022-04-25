/**
 * @file undistort.cpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief This script is to test the undistortion of images using the camera intrinsics and 
 * @version 0.1
 * @date 2021-08-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <opencv2/opencv.hpp>
int main(int argc, char** argv) {
    cv::Mat input = cv::imread("./test.png");
    if (input.empty()) {
        std::cerr << "Failure to load image" << std::endl;
        exit(0);
    }
    cv::Mat M(3, 3, CV_32FC1);
    M.at<float>(0, 0) = 635.9; M.at<float>(0, 1) = 0;       M.at<float>(0, 2) = 643.99;
    M.at<float>(1, 0) = 0;     M.at<float>(1, 1) = 635.15;  M.at<float>(1, 2) = 381.75; 
    M.at<float>(2, 0) = 0;     M.at<float>(2, 1) = 0;       M.at<float>(2, 2) = 1;
    std::cout << "Matrix intrinsics is : \n";
    std::cout << M << std::endl;  
    cv::Mat distortion_coeffs(1, 5, CV_32FC1);
    M.at<float>(0, 0) = -0.0552;
    M.at<float>(0, 1) = 0.0620;
    M.at<float>(0, 2) = -0.00019;
    M.at<float>(0, 3) = 0.00035;
    M.at<float>(0, 4) = -0.019329;
    std::cout << "The undistortion coefficient is: " << std::endl;
    std::cout << std::setprecision(3) << M.at<float>(0, 0) << std::endl;

    cv::Mat undistorted_image;
    cv::undistort(input, undistorted_image, M, distortion_coeffs);
    cv::imshow("original", input);
    cv::imshow("undistorted", undistorted_image);
    cv::waitKey(0);

}