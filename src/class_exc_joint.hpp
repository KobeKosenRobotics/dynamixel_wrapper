#ifndef CLASS_EXC_JOINT_HPP
#define CLASS_EXC_JOINT_HPP

#include "class_exc_arm_property.hpp"

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

class ExCJoint
{
    private:
        int _joint;
        Eigen::Matrix<double, 3, 1> _q, _v, _w;
        double _cos_theta, _sin_theta, _v_theta;
        Eigen::Matrix<double, 3, 3> _exp_w_hat_theta;
        Eigen::Matrix<double, 3, 1> _position_exp_xi_hat_theta;
        Eigen::Matrix<double, 3, 3> _eye3;
        Eigen::Matrix<double, 4, 4> _exp_xi_hat_theta;
        Eigen::Matrix<double, 6, 1> _xi;
        Eigen::Matrix<double, 3, 1> _position_xi;

    public:
        ExCJoint();
        void setJoint(int joint);

        Eigen::Matrix<double, 4, 4> getExpXiHatTheta(double angle);
            Eigen::Matrix<double, 3, 3> getExpWHatTheta(double angle);
                double getVTheta(double angle);
                    double getCosTheta(double angle);
                    double getSinTheta(double angle);
        Eigen::Matrix<double, 6, 1> getXi();
};

// Constructor
ExCJoint::ExCJoint()
{}

void ExCJoint::setJoint(int joint)
{
    _joint = joint;

    _q = exc_arm_property.getQ(joint);
    // _v = exc_arm_property.getV(joint);
    _w = exc_arm_property.getW(joint);
    _v = -_w.cross(_q);

    _eye3.setIdentity();
}

Eigen::Matrix<double, 4, 4> ExCJoint::getExpXiHatTheta(double angle)
{
    getCosTheta(angle);
    getSinTheta(angle);
    getVTheta(angle);
    getExpWHatTheta(angle);

    _position_exp_xi_hat_theta = ((_eye3-_exp_w_hat_theta)*(_w.cross(_v))) + (_w*_w.transpose())*_v*angle;

    _exp_xi_hat_theta <<
    _exp_w_hat_theta, _position_exp_xi_hat_theta,
    0.0, 0.0, 0.0, 1.0;

    return _exp_xi_hat_theta;
}

Eigen::Matrix<double, 3, 3> ExCJoint::getExpWHatTheta(double angle)
{
    _exp_w_hat_theta(0,0) = pow(_w(0,0),2)*_v_theta + _cos_theta;
    _exp_w_hat_theta(0,1) = _w(0,0)*_w(1,0)*_v_theta - _w(2,0)*_sin_theta;
    _exp_w_hat_theta(0,2) = _w(0,0)*_w(2,0)*_v_theta + _w(1,0)*_sin_theta;

    _exp_w_hat_theta(1,0) = _w(0,0)*_w(1,0)*_v_theta + _w(2,0)*_sin_theta;
    _exp_w_hat_theta(1,1) = pow(_w(1,0),2)*_v_theta + _cos_theta;
    _exp_w_hat_theta(1,2) = _w(1,0)*_w(2,0)*_v_theta - _w(0,0)*_sin_theta;

    _exp_w_hat_theta(2,0) = _w(0,0)*_w(2,0)*_v_theta - _w(1,0)*_sin_theta;
    _exp_w_hat_theta(2,1) = _w(1,0)*_w(2,0)*_v_theta + _w(0,0)*_sin_theta;
    _exp_w_hat_theta(2,2) = pow(_w(2,0),2)*_v_theta + _cos_theta;

    return _exp_w_hat_theta;
}

double ExCJoint::getVTheta(double angle)
{
    _v_theta = 1-_cos_theta;
    return _v_theta;
}

double ExCJoint::getCosTheta(double angle)
{
    _cos_theta = cos(angle);
    return _cos_theta;
}

double ExCJoint::getSinTheta(double angle)
{
    _sin_theta = sin(angle);
    return _sin_theta;
}

Eigen::Matrix<double, 6, 1> ExCJoint::getXi()
{
    _xi <<
    -_w.cross(_q),
    _w;

    return _xi;
}

#endif