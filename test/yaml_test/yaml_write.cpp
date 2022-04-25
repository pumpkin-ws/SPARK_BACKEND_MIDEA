#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
    cv::FileStorage fs_reader("./test.yml", cv::FileStorage::READ);
    std::cout << (std::string)fs_reader["name"] << std::endl;
    std::cout << (double)fs_reader["pi"] << std::endl;
    return 0;
}