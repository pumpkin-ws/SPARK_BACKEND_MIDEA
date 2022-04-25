#include "yaml-cpp/yaml.h"
#include <iostream>

std::string getSparkDir() {
    const char* home = getenv("HOME");
    std::string home_dir(home);
    home_dir += "/Documents/spark";
    return home_dir;
};

int main(int argc, char** argv) {
    std::string home = getSparkDir();
    std::string wifi_tcp0 = home + "/robot/gripper/wifi2020/tcp0.yml";
    YAML::Node data_reader = YAML::LoadFile(wifi_tcp0);
    std::cout << "x:" << data_reader["x"].as<double>() << std::endl;
    std::cout << "y:" << data_reader["y"].as<double>() << std::endl;
    std::cout << "z:" << data_reader["z"].as<double>() << std::endl;
    std::cout << "rx:" << data_reader["rx"].as<double>() << std::endl;
    std::cout << "ry:" << data_reader["ry"].as<double>() << std::endl;
    std::cout << "rz:" << data_reader["rz"].as<double>() << std::endl;
    return EXIT_SUCCESS;
}