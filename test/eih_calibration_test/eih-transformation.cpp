/**
 * @file eih_result_trans.cpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief Transformation of 
 * @version 0.1
 * @date 2021-08-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "math/include/coord_transform.hpp"

int main (int argc, char** argv) {
    Eigen::Vector3f pc_point(0.0593, -0.028, 0.363);
    Eigen::Matrix4f hand2eye = Eigen::Matrix4f::Identity();
    hand2eye <<    0.02807,	0.00159,	0.9996,	-0.008573,	
0.9995,	-0.01365,	-0.02805,	-0.01952,	
0.0136,	0.9999,	-0.001972,	0.1194,	
0,	0,	0,	1;

    Eigen::Isometry3f iso_hand2eye(hand2eye);

    std::cout << "The hand-eye transformation matrix is: " << std::endl;
    std::cout << hand2eye << std::endl;
    Eigen::Matrix4f depth2color = Eigen::Matrix4f::Identity();
    depth2color <<   0.999997,         0.00130644,       0.00211002,    -0.0591932386159897,
                    -0.0013171,        0.999986,         0.00505852,     0.000186097793630324, 
                    -0.00210338,      -0.00506128,       0.999985,       0.000512324157170951,  
                     0,                0,                0,              1;
    Eigen::Isometry3f iso_depth2color(depth2color);
    
    Eigen::Isometry3f tool_pose = Eigen::Isometry3f::Identity();
    tool_pose.pretranslate(Eigen::Vector3f(0.683, 0.178, 0.238));
    tool_pose.rotate(spark_math::eulerToQuaternion(Eigen::Vector3f(
        spark_math::deg2rad(-87.13),
        spark_math::deg2rad(3.043),
        spark_math::deg2rad(-80.698)
    )));

    Eigen::Vector3f point_in_base = tool_pose * iso_hand2eye.inverse() * iso_depth2color * pc_point;
    std::cout << "The point in base is :" << std::endl;
    std::cout << point_in_base << std::endl;
    return EXIT_SUCCESS;

}