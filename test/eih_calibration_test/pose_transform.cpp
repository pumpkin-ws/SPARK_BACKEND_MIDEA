#include "coord_transform.hpp"
#include <iostream>

int main (int argc, char** argv) {
    double x, y, z, rx, ry, rz;
    std::cout << "Input x: ";  
    std::cin >> x;
    std::cout << "Input y: ";  
    std::cin >> y;
    std::cout << "Input z: ";  
    std::cin >> z;
    std::cout << "Input rx: ";  
    std::cin >> rx;
    std::cout << "Input ry: ";  
    std::cin >> ry;
    std::cout << "Input rz: ";  
    std::cin >> rz;

    rx = spark_math::deg2rad(rx);
    ry = spark_math::deg2rad(ry);
    rz = spark_math::deg2rad(rz);

    Eigen::Quaternionf quat = spark_math::eulerToQuaternion(Eigen::Vector3f(rx, ry, rz));
    Eigen::Isometry3f h2e = Eigen::Isometry3f::Identity();
    h2e.pretranslate(Eigen::Vector3f(x, y, z));
    h2e.rotate(quat);
    std::cout << "The transformation matrix is: " << std::endl;
    std::cout << h2e.matrix() << std::endl; 

    printf("In yaml usable format:\n");
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << " - " << h2e.matrix()(i, j) << std::endl;
        }
    }
}