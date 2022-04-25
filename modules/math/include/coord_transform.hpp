#ifndef COORD_TRANSFORM_HPP
#define COORD_TRANSFORM_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>

/**
 * @brief 
 * The default angle representation is radian.
 * 
 */
namespace spark_math {
/**
 * @brief Transformation from Euler angles to quaternion, the default order is zyx
 * 
 * @param euler_angles : the order of the euler angle should be rx, then ry, then rz
 * @param rotation_order : using a string to define the transformation order
 * @return Eigen::Quaternionf 
 */
Eigen::Quaternionf eulerToQuaternion(Eigen::Vector3f euler_angles, std::string rotation_order = "zyx");
/**
 * @brief Transformation from quaternion to euler angles, the default order is zyx
 * 
 * @param quat : the input quaternion
 * @param rotation_order : using a string to define the transformation order 
 * @return Eigen::Vector3f : the result euler angles in the order or rx, ry, rz (in radians)
 */
Eigen::Vector3f quaternionToEuler(Eigen::Quaternionf quat, std::string rotation_order = "zyx");

double deg2rad(double degree);

double rad2deg(double radian);

Eigen::Quaternionf Euler_to_Quaternion(Eigen::Vector3f euler);

}

#endif