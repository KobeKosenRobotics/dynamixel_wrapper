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

const int JOINT_NUMBER = 32;
const int CHAIN_NUMBER = 5;

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

    public:
        // Constructor
        TreeExCArmProperty();

        // Joint Parameter
        Eigen::Matrix<double, 3, 1> getQ(const int &joint_);
        Eigen::Matrix<double, 3, 1> getV(const int &joint_);
        Eigen::Matrix<double, 3, 1> getW(const int &joint_);

        // Joint Link
        Eigen::Matrix<double, 3, 1> getLink(int joint_);
        double getLink(int joint_, int axis_);

        // Joint Name
        std::string getJointName(int joint_);

        // Chain Matrix
        bool getChainMatrix(const int &chain_, const int &joint_);
        void setToolDefaultPose();
        Eigen::Matrix<double, 6, 1> getToolDefaultPose(int chain_);

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> getProportionalGainAngleOperating();
        double getProportionalGainExC();

        // Safety
        Eigen::Matrix<double, JOINT_NUMBER, 1> getInitialTargetAngle();
        double getLowerAngleLimit(int joint_);
        double getUpperAngleLimit(int joint_);
};
TreeExCArmProperty tree_property;

TreeExCArmProperty::TreeExCArmProperty()
{
    for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    {
        // _link(i,2) = double(1000.0/(JOINT_NUMBER+1));
        _link(i,2) = 300.0;

        if(i < JOINT_NUMBER)
        {
            if(i%2 == 0)
            {
                _rotation_axis(2,i) = 1.0;
            }
            else
            {
                _rotation_axis(1,i) = 1.0;
                _initial_target_angle(i,0) = 0.1;
            }

            _lower_angle_limit(i,0) = -M_PI;
            _upper_angle_limit(i,0) = M_PI;
        }

        std::stringstream ss;
        ss << "joint" << i;
        _joint_name(i,0) = ss.str();
    }

    _proportional_gain_angle_operating.setIdentity();

    // Chain Matrix
    // - 0- 1- 2- 3- 4- 5- 6- 7-32
    //       - 8- 9-10-11-12-13-33
    //       -14-15-16-17-18-19-34
    //       -20-21-22-23-24-25-35
    //       -26-27-28-29-30-31-36
    _chain_matrix <<
    // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1;

    setToolDefaultPose();
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
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getLink(const int joint_)
{
    return (_link.row(joint_)).transpose();
}

double TreeExCArmProperty::getLink(int joint_, int axis_)
{
    return _link(joint_,axis_);
}
// Joint Name
std::string TreeExCArmProperty::getJointName(int joint_)
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

Eigen::Matrix<double, 6, 1> TreeExCArmProperty::getToolDefaultPose(int chain_)
{
    return _tool_default_pose.col(chain_);
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

double TreeExCArmProperty::getLowerAngleLimit(int joint_)
{
    return _lower_angle_limit(joint_,0);
}

double TreeExCArmProperty::getUpperAngleLimit(int joint_)
{
    return _upper_angle_limit(joint_,0);
}

#endif