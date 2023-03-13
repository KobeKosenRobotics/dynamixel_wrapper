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

const int JOINT_NUMBER = 14;
const int CHAIN_NUMBER = 2;

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class TreeExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> _joint_position, _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+CHAIN_NUMBER, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;
        double _proportional_gain_exc = 20.0;
        Eigen::Matrix<double, JOINT_NUMBER, 1> _initial_target_angle, _lower_angle_limit, _upper_angle_limit;
        Eigen::Matrix<bool, CHAIN_NUMBER, JOINT_NUMBER> _chain_matrix;
        Eigen::Matrix<double, CHAIN_NUMBER, 6> _tool_default_pose;

    public:
        // Constructor
        TreeExCArmProperty();

        // Link to Joint Position
        Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> link2JointPosition(Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> link_);

        // Joint Parameter
        Eigen::Matrix<double, 3, 1> getQ(int &joint_);
        Eigen::Matrix<double, 3, 1> getV(int &joint_);
        Eigen::Matrix<double, 3, 1> getW(int &joint_);

        // Joint Link
        Eigen::Matrix<double, 3, 1> getLink(int joint_);
        double getLink(int joint_, int axis_);

        // Joint Name
        std::string getJointName(int joint_);

        // Chain Matrix
        bool getChainMatrix(int &chain_, int &joint_);
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

    _joint_position = link2JointPosition(_link);

    // Chain Matrix
    // - 0- 1- 2- 3- 4- 5- 6- 7-14
    //       - 8- 9-10-11-12-13-15
    _chain_matrix <<
    // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1;

    _tool_default_pose <<
     1, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0;
}

// Link to Joint Position
Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> TreeExCArmProperty::link2JointPosition(Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> link_)
{
    for(int column = 0; column < 3; column++)
    {
        for(int row = 1; row < (JOINT_NUMBER+CHAIN_NUMBER); row++)
        {
            link_(row,column) += link_(row-1,column);
        }
    }

    return link_.transpose();
}

// Joint Parameter
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getQ(int &joint_)
{
    return _joint_position.col(joint_);
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getV(int &joint_)
{
    return _translation_axis.col(joint_);
}

Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getW(int &joint_)
{
    return _rotation_axis.col(joint_);
}

// Joint Link
Eigen::Matrix<double, 3, 1> TreeExCArmProperty::getLink(int joint_)
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
bool TreeExCArmProperty::getChainMatrix(int &chain_, int &joint_)
{
    return _chain_matrix(chain_, joint_);
}

Eigen::Matrix<double, 6, 1> TreeExCArmProperty::getToolDefaultPose(int chain_)
{
    Eigen::Matrix<double, 6, 1> pose_;
    pose_ <<
    _tool_default_pose(chain_,0),
    _tool_default_pose(chain_,1),
    _tool_default_pose(chain_,2),
    _tool_default_pose(chain_,3),
    _tool_default_pose(chain_,4),
    _tool_default_pose(chain_,5);

    return pose_;
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