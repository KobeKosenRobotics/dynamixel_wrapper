#ifndef CLASS_TREE_EXC_ARM_BASE_HPP
#define CLASS_TREE_EXC_ARM_BASE_HPP

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
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

class TreeExCArmBase
{
    private:
        /* data */
    public:
        // Constructor
        TreeExCArmBase();

        Eigen::Matrix<double, 3, 3> getIdentity3();
        Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> &vector_);

        // Rotation Matrix
        Eigen::Matrix<double, 3, 3> getRotationMatrixX(double &angle_);
        Eigen::Matrix<double, 3, 3> getRotationMatrixY(double &angle_);
        Eigen::Matrix<double, 3, 3> getRotationMatrixZ(double &angle_);

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> getPose(Eigen::Matrix<double, 4, 4> homogeneous_transformation_matrix_);

        Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 4, 4> &matrix_);
        // TODO: adjointInverse(Matrix &matrix_); comparison calculation time
        Eigen::Matrix<double, 6, 6> adjointInverse(Eigen::Matrix<double, 4, 4> matrix_);
};
TreeExCArmBase tree_base;

// Constructor
TreeExCArmBase::TreeExCArmBase()
{
}

// Identity
Eigen::Matrix<double, 3, 3> TreeExCArmBase::getIdentity3()
{
    Eigen::Matrix<double, 3, 3> identity_;
    identity_.setIdentity();
    return identity_;
}

//
Eigen::Matrix<double, 3, 3> TreeExCArmBase::hat(Eigen::Matrix<double, 3, 1> &vector_)
{
    Eigen::Matrix<double, 3, 3> hat_vector_;

    hat_vector_ <<
              0.0, -vector_(2,0),  vector_(1,0),
     vector_(2,0),           0.0, -vector_(0,0),
    -vector_(1,0),  vector_(0,0),           0.0;

    return hat_vector_;
}

// Rotation Matrix
Eigen::Matrix<double, 3, 3> TreeExCArmBase::getRotationMatrixX(double &angle_)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_x_;
    rotation_matrix_x_ <<
    1.0,         0.0,          0.0,
    0.0, cos(angle_), -sin(angle_),
    0.0, sin(angle_),  cos(angle_);
    return rotation_matrix_x_;
}

Eigen::Matrix<double, 3, 3> TreeExCArmBase::getRotationMatrixY(double &angle_)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_y_;
    rotation_matrix_y_ <<
     cos(angle_),        0.0, sin(angle_),
             0.0,        1.0,         0.0,
    -sin(angle_),        0.0, cos(angle_);
    return rotation_matrix_y_;
}

Eigen::Matrix<double, 3, 3> TreeExCArmBase::getRotationMatrixZ(double &angle_)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_z_;
    rotation_matrix_z_ <<
    cos(angle_), -sin(angle_), 0.0,
    sin(angle_),  cos(angle_), 0.0,
            0.0,          0.0, 1.0;
    return rotation_matrix_z_;
}

// Forward Kinematics
Eigen::Matrix<double, 6, 1> TreeExCArmBase::getPose(Eigen::Matrix<double, 4, 4> homogeneous_transformation_matrix_)
{
    Eigen::Matrix<double, 6, 1> pose_;

    pose_.block(0,0,3,1) = homogeneous_transformation_matrix_.block(0,3,3,1);

    pose_(4,0) = -asin(homogeneous_transformation_matrix_(2,0));
    pose_(3,0) = acos(homogeneous_transformation_matrix_(0,0)/cos(pose_(4,0)));
    if(homogeneous_transformation_matrix_(1,0)/cos(pose_(4,0)) < 0) pose_(3,0) *= (-1);
    pose_(5,0) = acos(homogeneous_transformation_matrix_(2,2)/cos(pose_(4,0)));
    if(homogeneous_transformation_matrix_(2,1)/cos(pose_(4,0)) < 0) pose_(5,0) *=(-1);

    for(int i = 3; i < 6; i++)
    {
        if(isnan(pose_(i,0))) pose_(i,0) = 0.0;
    }

    return pose_;
}

// Adjoint
Eigen::Matrix<double, 6, 6> TreeExCArmBase::adjoint(Eigen::Matrix<double, 4, 4> &matrix_)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_matrix_;

    rotation_matrix_ <<
    matrix_(0,0), matrix_(0,1), matrix_(0,2),
    matrix_(1,0), matrix_(1,1), matrix_(1,2),
    matrix_(2,0), matrix_(2,1), matrix_(2,2);

    position_ <<
    matrix_(0,3),
    matrix_(1,3),
    matrix_(2,3);

    adjoint_matrix_ <<
                       rotation_matrix_, tree_base.hat(position_)*rotation_matrix_,
    Eigen::Matrix<double, 3, 3>::Zero(),                          rotation_matrix_;

    return adjoint_matrix_;
}

Eigen::Matrix<double, 6, 6> TreeExCArmBase::adjointInverse(Eigen::Matrix<double, 4, 4> matrix_)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_inverse_matrix_;

    rotation_matrix_ <<
    matrix_(0,0), matrix_(0,1), matrix_(0,2),
    matrix_(1,0), matrix_(1,1), matrix_(1,2),
    matrix_(2,0), matrix_(2,1), matrix_(2,2);

    position_ <<
    matrix_(0,3),
    matrix_(1,3),
    matrix_(2,3);

    adjoint_inverse_matrix_ <<
           rotation_matrix_.transpose(), -rotation_matrix_.transpose()*tree_base.hat(position_),
    Eigen::Matrix<double, 3, 3>::Zero(),                           rotation_matrix_.transpose();

    return adjoint_inverse_matrix_;
}

#endif