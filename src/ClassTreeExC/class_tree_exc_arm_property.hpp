#ifndef CLASS_TREE_EXC_ARM_PROPERTY_HPP
#define CLASS_TREE_EXC_ARM_PROPERTY_HPP

#include "class_tree_exc_arm_base.hpp"

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

// const int JOINT_NUMBER = 32;
// const int CHAIN_NUMBER = 5;
// const int BINDING_CONDITIONS = 3;    // 1: Frictionless point contact    3: Point contact with friction    4: Soft finger

const int JOINT_NUMBER = 29;
const int CHAIN_NUMBER = 5;
const int BINDING_CONDITIONS = 3;

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class TreeExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> _joint_position, _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+CHAIN_NUMBER, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;
        double _proportional_gain_exc = 2.0;
        Eigen::Matrix<double, JOINT_NUMBER, 1> _initial_target_angle, _lower_angle_limit, _upper_angle_limit;
        Eigen::Matrix<bool, CHAIN_NUMBER, JOINT_NUMBER> _chain_matrix;
        Eigen::Matrix<double, 6, CHAIN_NUMBER> _tool_default_pose;
        Eigen::Matrix<double, BINDING_CONDITIONS, 6> _binding_conditions_matrix;
        Eigen::Matrix<double, BINDING_CONDITIONS*CHAIN_NUMBER, 6*CHAIN_NUMBER> _binding_matrix;

    public:
        // Constructor
        TreeExCArmProperty();

        // Joint Parameter
        Eigen::Matrix<double, 3, 1> getQ(const int &joint_);
        Eigen::Matrix<double, 3, 1> getV(const int &joint_);
        Eigen::Matrix<double, 3, 1> getW(const int &joint_);

        // Joint Link
        Eigen::Matrix<double, 3, 1> getLink(const int &joint_);
        double getLink(const int &joint_, const int &axis_);

        // Joint Name
        std::string getJointName(const int &joint_);

        // Chain Matrix
        bool getChainMatrix(const int &chain_, const int &joint_);
        void setToolDefaultPose();
        Eigen::Matrix<double, 6, 1> getToolDefaultPose(const int &chain_);

        // Binding Conditions
        void setBindingMatrix(const int &binding_conditions_);
        Eigen::Matrix<double, BINDING_CONDITIONS*CHAIN_NUMBER, 6*CHAIN_NUMBER> getBindingMatrix();

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> getProportionalGainAngleOperating();
        double getProportionalGainExC();

        // Safety
        Eigen::Matrix<double, JOINT_NUMBER, 1> getInitialTargetAngle();
        double getLowerAngleLimit(const int &joint_);
        double getUpperAngleLimit(const int &joint_);
};
TreeExCArmProperty tree_property;

TreeExCArmProperty::TreeExCArmProperty()
{
    // for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    // {
    //     // _link(i,2) = double(1000.0/(JOINT_NUMBER+1));
    //     _link(i,2) = 300.0;

    //     if(i < JOINT_NUMBER)
    //     {
    //         if(i%2 == 0)
    //         {
    //             _rotation_axis(2,i) = 1.0;
    //         }
    //         else
    //         {
    //             _rotation_axis(1,i) = 1.0;
    //             _initial_target_angle(i,0) = 0.1;
    //         }

    //         _lower_angle_limit(i,0) = -M_PI;
    //         _upper_angle_limit(i,0) = M_PI;
    //     }

    //     std::stringstream ss;
    //     ss << "joint" << i;
    //     _joint_name(i,0) = ss.str();
    // }

    for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    {
        if(i < JOINT_NUMBER)
        {
            _lower_angle_limit(i,0) = -M_PI;
            _upper_angle_limit(i,0) = M_PI;
        }

        std::stringstream ss;
        ss << "joint" << i;
        _joint_name(i,0) = ss.str();
    }

    _link <<
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,

    0, 0, 300,

    0, 0, 300,
    0, 0, 0,
    0, 0, 0,

    -34, 0, 29,
    0, 0, 0,
    -38*cos(45*deg2rad), 0, 38*sin(45*deg2rad),
    0, 0, 0,
    -32*cos(45*deg2rad), 0, 32*sin(45*deg2rad),

    -34, 0, 95,
    0, 0, 0,
    0, 0, 45,
    0, 0, 25,

    -11, 0, 99,
    0, 0, 0,
    0, 0, 45,
    0, 0, 25,

    11, 0, 95,
    0, 0, 0,
    0, 0, 45,
    0, 0, 25,

    22, 0, 20.71,
    0, 0, 86.6-20.71,
    0, 0, 0,
    0, 0, 45,
    0, 0, 25,

    -27.5*cos(45*deg2rad), 0, 27.5*sin(45*deg2rad),
    0, 0, 26,
    0, 0, 26,
    0, 0, 26,
    0, 0, 26;

    _proportional_gain_angle_operating.setIdentity();

    // Chain Matrix
    // - 0- 1- 2- 3- 4- 5- 6- 7-32
    //       - 8- 9-10-11-12-13-33
    //       -14-15-16-17-18-19-34
    //       -20-21-22-23-24-25-35
    //       -26-27-28-29-30-31-36
    // _chain_matrix <<
    // // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
    // 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    // 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1;

    _chain_matrix <<
    // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1;

    Eigen::Matrix<double, 34, 3> mat_;
    mat_ <<
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,

    1, 0, 0,

    1, 0, 0,
    0, 1, 0,
    0, 0, 1,

    -cos(45*deg2rad), 0, -cos(45*deg2rad),
    -cos(45*deg2rad), 0, cos(45*deg2rad),
    -cos(45*deg2rad), 0, -cos(45*deg2rad),
    0, 1, 0,
    0, 1, 0,

    1, 0, 0,
    0, 1, 0,
    1, 0, 0,
    1, 0, 0,

    1, 0, 0,
    0, 1, 0,
    1, 0, 0,
    1, 0, 0,

    1, 0, 0,
    0, 1, 0,
    1, 0, 0,
    1, 0, 0,


    -cos(55*deg2rad), 0, sin(55*deg2rad),
    1, 0, 0,
    0, 1, 0,
    1, 0, 0,
    1, 0, 0,

    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0;

    _rotation_axis = mat_.transpose();

    // setToolDefaultPose();
    _tool_default_pose <<
    -1  , -0.9, -0.3,  0.3,  0.9,
    -1  ,  0.8,  1  ,  1  ,  0.6,
     0  ,  0  ,  0  ,  0  ,  0  ,
     0  ,  0  ,  0  ,  0  ,  0  ,
     0  ,  0  ,  0  ,  0  ,  0  ,
     0  ,  0  ,  0  ,  0  ,  0  ;

    // [0.1, 0.1, 0.1, -1.0, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1]

    setBindingMatrix(BINDING_CONDITIONS);
}

// Joint Parameter
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getQ(const int &joint_)
{
    return _joint_position.col(joint_);
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getV(const int &joint_)
{
    return _translation_axis.col(joint_);
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getW(const int &joint_)
{
    return _rotation_axis.col(joint_);
}

// Joint Link
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getLink(const int &joint_)
{
    return (_link.row(joint_)).transpose();
}

double TreeExCArmProperty::getLink(const int &joint_, const int &axis_)
{
    return _link(joint_,axis_);
}
// Joint Name
std::string TreeExCArmProperty::getJointName(const int &joint_)
{
    return _joint_name(joint_,0);
}

// Chain Matrix
bool TreeExCArmProperty::getChainMatrix(const int &chain_, const int &joint_)
{
    if(joint_ < 0 || (JOINT_NUMBER+CHAIN_NUMBER) <= joint_)
    {
        return false;
    }

    if(JOINT_NUMBER <= joint_)
    {
        if((joint_-JOINT_NUMBER) == chain_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return _chain_matrix(chain_, joint_);
}

void TreeExCArmProperty::setToolDefaultPose()
{
    Eigen::Matrix<double, 6, 1> basic_vector_;
    basic_vector_ << 1, 0, 0, 0, 0, 0;

    for(int i = 0; i < CHAIN_NUMBER; i++)
    {
        _tool_default_pose.block(0,i,3,1) = tree_base.getRotationMatrixZ(2*M_PI*i/CHAIN_NUMBER)*basic_vector_.block(0,0,3,1);
    }
}

Eigen::Matrix<double, 6, 1> TreeExCArmProperty::getToolDefaultPose(const int &chain_)
{
    return _tool_default_pose.col(chain_);
}

// Binding Conditions
void TreeExCArmProperty::setBindingMatrix(const int &binding_conditions_)
{
    if(binding_conditions_ == 1)
    {
        _binding_conditions_matrix(0,2) = 1;
    }
    else if(binding_conditions_ == 3)
    {
        _binding_conditions_matrix(0,0) = 1;
        _binding_conditions_matrix(1,1) = 1;
        _binding_conditions_matrix(2,2) = 1;
    }
    else if(binding_conditions_ == 4)
    {
        _binding_conditions_matrix(0,0) = 1;
        _binding_conditions_matrix(1,1) = 1;
        _binding_conditions_matrix(2,2) = 1;
        _binding_conditions_matrix(3,5) = 1;
    }

    for(int i = 0; i < CHAIN_NUMBER; i++)
    {
        _binding_matrix.block(BINDING_CONDITIONS*i,6*i,BINDING_CONDITIONS,6) = _binding_conditions_matrix;
    }
}

Eigen::Matrix<double, BINDING_CONDITIONS*CHAIN_NUMBER, 6*CHAIN_NUMBER> TreeExCArmProperty::getBindingMatrix()
{
    return _binding_matrix;
}

// Gain
Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> TreeExCArmProperty::getProportionalGainAngleOperating()
{
    return _proportional_gain_angle_operating;
}
double TreeExCArmProperty::getProportionalGainExC()
{
    return _proportional_gain_exc;
}

// Safety
Eigen::Matrix<double, JOINT_NUMBER, 1> TreeExCArmProperty::getInitialTargetAngle()
{
    return _initial_target_angle;
}

double TreeExCArmProperty::getLowerAngleLimit(const int &joint_)
{
    return _lower_angle_limit(joint_,0);
}

double TreeExCArmProperty::getUpperAngleLimit(const int &joint_)
{
    return _upper_angle_limit(joint_,0);
}

#endif