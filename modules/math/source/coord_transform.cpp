#include "coord_transform.hpp"

Eigen::Quaternionf spark_math::eulerToQuaternion(Eigen::Vector3f euler_angles, std::string rotation_order) {
    if (rotation_order == "zyx" || rotation_order == "ZYX") {
        Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
        rotation_matrix = Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) * 
                          Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX());
        return Eigen::Quaternionf(rotation_matrix);
    } else if (rotation_order == "xyz" || rotation_order == "XYZ") {
        Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
        rotation_matrix = Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX()) * 
                          Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ());
        return Eigen::Quaternionf(rotation_matrix);
    } else {
        printf("No valid key detected. Using the default transformation in zyx order.\n");
        Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
        rotation_matrix = Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) * 
                          Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX());
        return Eigen::Quaternionf(rotation_matrix);        
    }
};

Eigen::Vector3f spark_math::quaternionToEuler(Eigen::Quaternionf quat, std::string rotation_order) {
    if (rotation_order == "zyx" || rotation_order == "ZYX") {
        Eigen::Vector3f euler_angles = Eigen::Vector3f::Zero();
        // for rotation order of 2, 1, 0, the last element is rx and the first element is rz
        euler_angles[0] = quat.matrix().eulerAngles(2, 1, 0)[2];
        euler_angles[1] = quat.matrix().eulerAngles(2, 1, 0)[1];
        euler_angles[2] = quat.matrix().eulerAngles(2, 1, 0)[0];
        return euler_angles;
    } else if (rotation_order == "xyz" || rotation_order == "XYZ") {
        Eigen::Vector3f euler_angles = Eigen::Vector3f::Zero();
        // TODO: does the transformation in
        euler_angles[2] = quat.matrix().eulerAngles(0, 1, 2)[0]; // 0, 1, 2 is the eigen convention for transformation in xyz order
        euler_angles[1] = quat.matrix().eulerAngles(0, 1, 2)[1];
        euler_angles[0] = quat.matrix().eulerAngles(0, 1, 2)[2];
        return euler_angles;
    } else {
        printf("No valid key detected. Using the default transformation in zyx order.\n");
        Eigen::Vector3f euler_angles = Eigen::Vector3f::Zero();
        euler_angles[0] = quat.matrix().eulerAngles(2, 1, 0)[2];
        euler_angles[1] = quat.matrix().eulerAngles(2, 1, 0)[1];
        euler_angles[2] = quat.matrix().eulerAngles(2, 1, 0)[0];
        return euler_angles;
    }
}

double spark_math::deg2rad(double deg) {
    return deg * M_PI / 180.0;
};

double spark_math::rad2deg(double radian) {
    return radian * 180.0 / M_PI;
};

Eigen::Quaternionf spark_math::Euler_to_Quaternion(Eigen::Vector3f euler)
{
    
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf(euler(2), Eigen::Vector3f(0,0,1))*
      Eigen::AngleAxisf(euler(1), Eigen::Vector3f(0,1,0))*
      Eigen::AngleAxisf(euler(0), Eigen::Vector3f(1,0,0));
  return Eigen::Quaternionf(R);
}