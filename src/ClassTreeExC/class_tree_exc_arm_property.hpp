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

const int JOINT_NUMBER = 6;
const int CHAIN_NUMBER = 2;

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class TreeExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> _joint_position;
        Eigen::Matrix<double, 3, JOINT_NUMBER> _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+CHAIN_NUMBER, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;
        Eigen::Matrix<double, JOINT_NUMBER, 1> _initial_target_angle;
        Eigen::Matrix<double, JOINT_NUMBER, 2> _joint_angle_limit;
        Eigen::Matrix<bool, CHAIN_NUMBER, JOINT_NUMBER> _chain_matrix;

    public:
        // Constructor
        TreeExCArmProperty();

        // Joint Parameter
        Eigen::Matrix<double, 3, 1> getQ(int &joint_);
        Eigen::Matrix<double, 3, 1> getV(int &joint_);
        Eigen::Matrix<double, 3, 1> getW(int &joint_);

        // Chain Matrix
        bool getChainMatrix(int &chain_, int &joint_);
};
TreeExCArmProperty tree_property;

TreeExCArmProperty::TreeExCArmProperty()
{
}

// Joint Parameter
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getQ(int &joint_)
{
    Eigen::Matrix<double, 3, 1> q_;
    q_ << _joint_position(0,joint_), _joint_position(1,joint_), _joint_position(2,joint_);
    return q_;
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getV(int &joint_)
{
    Eigen::Matrix<double, 3, 1> v_;
    v_ << _translation_axis(0,joint_), _translation_axis(1,joint_), _translation_axis(2,joint_);
    return v_;
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getW(int &joint_)
{
    Eigen::Matrix<double, 3, 1> w_;
    w_ << _rotation_axis(0,joint_), _rotation_axis(1,joint_), _rotation_axis(2,joint_);
    return w_;
}

bool TreeExCArmProperty::getChainMatrix(int &chain_, int &joint_)
{
    return _chain_matrix(chain_, joint_);
}

#endif