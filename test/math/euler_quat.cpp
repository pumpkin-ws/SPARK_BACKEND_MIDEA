#include "math/include/coord_transform.hpp"
#include <iostream>

int main(int argc, char** argv) {
    Eigen::Vector3f euler_angles(0.4, 0.5, 1.0);
    std::cout << "The original angle is:" << std::endl;
    std::cout << euler_angles << std::endl;
    Eigen::Quaternionf quat = spark_math::eulerToQuaternion(euler_angles);
    printf("The calculated quaternion is : \n");
    std::cout << quat.w() << ", " <<quat.x() << ", " << quat.y() << ", " << quat.z() << "." <<  std::endl;
    printf("The quaternion to euler angle is : \n");
    std::cout << spark_math::quaternionToEuler(quat) << std::endl; 
}