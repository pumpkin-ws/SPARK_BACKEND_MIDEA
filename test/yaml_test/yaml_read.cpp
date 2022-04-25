#include "opencv2/opencv.hpp"
#include <string>

int main(int argc, char** argv) {
    cv::FileStorage fs_write("./test.yml", cv::FileStorage::WRITE);
    fs_write << "d" << 5.3;
    fs_write << "float" << 23.1231241341251345234;
    fs_write << "pi" << M_PI;
    fs_write << "name" << "Sheng Wei";
    fs_write.release();
    cv::FileStorage fs_read("./test.yml", cv::FileStorage::READ);
    std::cout << (double)fs_read["d"] << std::endl;
    std::cout << (double)fs_read["float"] << std::endl;
    std::cout << (std::string)fs_read["name"] << std::endl;
    fs_read.release();
}