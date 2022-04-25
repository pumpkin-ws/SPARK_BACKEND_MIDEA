#include "filesystem/include/file_system.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    std::cout << std::boolalpha << spark_filesystem::checkDirectoryExists("random_directory") << std::endl;
    spark_filesystem::createDirectory("random_directory");
    return EXIT_SUCCESS;
}