#ifndef CLASS_QUATERNION_HPP
#define CLASS_QUATERNION_HPP

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

class Quaternion
{
    private:
        /* data */
    public:
        Eigen::Matrix<double, 4, 1> getProduct(Eigen::Matrix<double, 4, 1> quaternion_a_, Eigen::Matrix<double, 4, 1> quaternion_b_);
        Eigen::Matrix<double, 4, 1> getInverse(Eigen::Matrix<double, 4, 1> quaternion_);
        Eigen::Matrix<double, 3, 3> quaternion2RotationMatrix(Eigen::Matrix<double, 4, 1> quaternion_);
        Eigen::Matrix<double, 4, 1> rotationMatrix2Quaternion(Eigen::Matrix<double, 3, 3> rotation_matrix_);
        Eigen::Matrix<double, 3, 1> quaternion2EulerAngle(Eigen::Matrix<double, 4, 1> quaternion_);
};
Quaternion quaternion;

Eigen::Matrix<double, 4, 1> Quaternion::getProduct(Eigen::Matrix<double, 4, 1> quaternion_a_, Eigen::Matrix<double, 4, 1> quaternion_b_)
{
    Eigen::Matrix<double, 4, 1> quaternion_c_;
    Eigen::Matrix<double, 4, 4> alternating_quaternion_a_;

    alternating_quaternion_a_ <<
     quaternion_a_(3,0), -quaternion_a_(2,0),  quaternion_a_(1,0),  quaternion_a_(0,0),
     quaternion_a_(2,0),  quaternion_a_(3,0), -quaternion_a_(0,0),  quaternion_a_(1,0),
    -quaternion_a_(1,0),  quaternion_a_(0,0),  quaternion_a_(3,0),  quaternion_a_(2,0),
    -quaternion_a_(0,0), -quaternion_a_(1,0), -quaternion_a_(2,0),  quaternion_a_(3,0);

    quaternion_c_ = alternating_quaternion_a_ * quaternion_b_;

    return quaternion_c_;
}

Eigen::Matrix<double, 4, 1> Quaternion::getInverse(Eigen::Matrix<double, 4, 1> quaternion_)
{
    Eigen::Matrix<double, 4, 1> inverse_quaternion_;
    inverse_quaternion_ << -quaternion_(0,0), -quaternion_(1,0), -quaternion_(2,0), quaternion_(3,0);
    return inverse_quaternion_;
}

Eigen::Matrix<double, 3, 3> Quaternion::quaternion2RotationMatrix(Eigen::Matrix<double, 4, 1> quaternion_)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;

    rotation_matrix_(0,0) = 1.0f - 2.0f * quaternion_(1,0)* quaternion_(1,0)- 2.0f * quaternion_(2,0) * quaternion_(2,0);
    rotation_matrix_(0,1) = 2.0f * quaternion_(0,0) * quaternion_(1,0)+ 2.0f * quaternion_(3,0) * quaternion_(2,0);
    rotation_matrix_(0,2) = 2.0f * quaternion_(0,0) * quaternion_(2,0) - 2.0f * quaternion_(3,0) * quaternion_(1,0);

    rotation_matrix_(1,0) = 2.0f * quaternion_(0,0) * quaternion_(1,0)- 2.0f * quaternion_(3,0) * quaternion_(2,0);
    rotation_matrix_(1,1) = 1.0f - 2.0f * quaternion_(0,0) * quaternion_(0,0) - 2.0f * quaternion_(2,0) * quaternion_(2,0);
    rotation_matrix_(1,2) = 2.0f * quaternion_(1,0)* quaternion_(2,0) + 2.0f * quaternion_(3,0) * quaternion_(0,0);

    rotation_matrix_(2,0) = 2.0f * quaternion_(0,0) * quaternion_(2,0) + 2.0f * quaternion_(3,0) * quaternion_(1,0);
    rotation_matrix_(2,1) = 2.0f * quaternion_(1,0)* quaternion_(2,0) - 2.0f * quaternion_(3,0) * quaternion_(0,0);
    rotation_matrix_(2,2) = 1.0f - 2.0f * quaternion_(0,0) * quaternion_(0,0) - 2.0f * quaternion_(1,0)* quaternion_(1,0);

    // rotation_matrix_(0,0) = 2.0*quaternion_(3,0)*quaternion_(3,0) + 2.0*quaternion_(0,0)*quaternion_(0,0) - 1.0;
    // rotation_matrix_(0,1) = 2.0*quaternion_(0,0)*quaternion_(1,0) - 2.0*quaternion_(2,0)*quaternion_(3,0);
    // rotation_matrix_(0,2) = 2.0*quaternion_(0,0)*quaternion_(2,0) + 2.0*quaternion_(1,0)*quaternion_(3,0);

    // rotation_matrix_(1,0) = 2.0*quaternion_(0,0)*quaternion_(1,0) + 2.0*quaternion_(2,0)*quaternion_(3,0);
    // rotation_matrix_(1,1) = 2.0*quaternion_(3,0)*quaternion_(3,0) + 2.0*quaternion_(1,0)*quaternion_(1,0) -1.0;
    // rotation_matrix_(1,2) = 2.0*quaternion_(1,0)*quaternion_(2,0) - 2.0*quaternion_(0,0)*quaternion_(3,0);

    // rotation_matrix_(2,0) = 2.0*quaternion_(0,0)*quaternion_(2,0) - 2.0*quaternion_(1,0)*quaternion_(3,0);
    // rotation_matrix_(2,1) = 2.0*quaternion_(1,0)*quaternion_(2,0) + 2.0*quaternion_(0,0)*quaternion_(3,0);
    // rotation_matrix_(2,2) = 2.0*quaternion_(3,0)*quaternion_(3,0) + 2.0*quaternion_(2,0)*quaternion_(2,0)-1;

    return rotation_matrix_;
}


Eigen::Matrix<double, 4, 1> Quaternion::rotationMatrix2Quaternion(Eigen::Matrix<double, 3, 3> rotation_matrix_)
{
    // 最大成分を検索
    double elem[4];
    elem[0] =  rotation_matrix_(0,0) - rotation_matrix_(1,1) - rotation_matrix_(2,2) + 1.0;
    elem[1] = -rotation_matrix_(0,0) + rotation_matrix_(1,1) - rotation_matrix_(2,2) + 1.0;
    elem[2] = -rotation_matrix_(0,0) - rotation_matrix_(1,1) + rotation_matrix_(2,2) + 1.0;
    elem[3] =  rotation_matrix_(0,0) + rotation_matrix_(1,1) + rotation_matrix_(2,2) + 1.0;

    int biggest_index_ = 0;
    for(int i = 1; i < 4; i++)
    {
        if(elem[i] > elem[biggest_index_])
        {
            biggest_index_ = i;
        }
    }

    // 最大要素の値を算出
    Eigen::Matrix<double, 4, 1> quaternion_;
    double v = sqrt(elem[biggest_index_]) * 0.5;
    quaternion_(biggest_index_, 0) = v;
    double mult = 0.25 / v;

    switch(biggest_index_)
    {
        case 0:
            quaternion_(1,0) = (rotation_matrix_(0,1) + rotation_matrix_(1,0)) * mult;
            quaternion_(2,0) = (rotation_matrix_(2,0) + rotation_matrix_(0,2)) * mult;
            quaternion_(3,0) = (rotation_matrix_(1,2) - rotation_matrix_(2,1)) * mult;
            break;

        case 1:
            quaternion_(0,0) = (rotation_matrix_(0,1) + rotation_matrix_(1,0)) * mult;
            quaternion_(2,0) = (rotation_matrix_(1,2) + rotation_matrix_(2,1)) * mult;
            quaternion_(3,0) = (rotation_matrix_(2,0) - rotation_matrix_(0,2)) * mult;
            break;

        case 2:
            quaternion_(0,0) = (rotation_matrix_(2,0) + rotation_matrix_(0,2)) * mult;
            quaternion_(1,0) = (rotation_matrix_(1,2) + rotation_matrix_(2,1)) * mult;
            quaternion_(3,0) = (rotation_matrix_(0,1) - rotation_matrix_(1,0)) * mult;
            break;

        case 3:
            quaternion_(0,0) = (rotation_matrix_(1,2) - rotation_matrix_(2,1)) * mult;
            quaternion_(1,0) = (rotation_matrix_(2,0) - rotation_matrix_(0,2)) * mult;
            quaternion_(2,0) = (rotation_matrix_(0,1) - rotation_matrix_(1,0)) * mult;
            break;
    }

    std::cout << "quaternion:" << std::endl << quaternion_ << std::endl;

    return quaternion_;
}

Eigen::Matrix<double, 3, 1> Quaternion::quaternion2EulerAngle(Eigen::Matrix<double, 4, 1> quaternion_)
{
    Eigen::Matrix<double, 3, 1> euler_;

    euler_(1,0) = asin(-2*(quaternion_(0,0)*quaternion_(2,0) - quaternion_(1,0)*quaternion_(3,0)));

    if(cos(euler_(1,0)) != 0)
    {
        euler_(2,0) = atan(2*(quaternion_(1,0)*quaternion_(2,0) + quaternion_(0,0)*quaternion_(3,0))/(2*(quaternion_(3,0)*quaternion_(3,0) + quaternion_(2,0)*quaternion_(2,0)) -1));
        euler_(0,0) = atan(2*(quaternion_(0,0)*quaternion_(1,0) + quaternion_(2,0)*quaternion_(3,0))/(2*(quaternion_(3,0)*quaternion_(3,0) + quaternion_(0,0)*quaternion_(0,0)) -1));
    }
    else
    {
        euler_(0,0) = atan(-(2*(quaternion_(0,0)*quaternion_(1,0) + quaternion_(2,0)*quaternion_(3,0))/(2*(quaternion_(3,0)*quaternion_(3,0) + quaternion_(0,0)*quaternion_(0,0)) -1)));
        euler_(2,0) = 0.0;
    }

    std::cout << "euler:" << std::endl << euler_ << std::endl;

    return euler_;
}

#endif