#ifndef CLASS_EXC_ARM_HPP
#define CLASS_EXC_ARM_HPP

#include "class_exc_arm_property.hpp"
#include "class_exc_joint.hpp"

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

class ExCArm
{
    private:
        // Joint
        ExCJoint exc_joint[JOINT_NUMBER];

        // Bool
        bool _motor_enable = false;
        bool _exc_enable = false, _exc_enable_old = false;
        bool _emergency_stop = false;

        // Motor
        Eigen::Matrix<double, JOINT_NUMBER, 1> _sensor_angle, _motor_angular_velocity;

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, 1> _target_angle;

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> _pose;
        Eigen::Matrix<double, 3, 1> _position, _euler;
        Eigen::Matrix<double, 3, 3> _rotation_all;    // _rotation_euler;

        // Inverse Kinematics
        Eigen::Matrix<double, 6, 1> _target_pose;
        double _proportional_gain_exc = 0.1;

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> /*_target_pose_mid,*/ _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 50;    // _liner_velocity[mm/s]

        // ExC (Exponential Coordinates)
        Eigen::Matrix<double, 6, JOINT_NUMBER> _exc_jacobian;
        Eigen::Matrix<double, 6, JOINT_NUMBER> _exc_jacobian_body;
        Eigen::Matrix<double, 6, 6> _transformation_matrix;
        Eigen::Matrix<double, 3, 3> _transformation_euler;

        // Other
        Eigen::Matrix<double, 3, 3> _zero;

    public:
        // Constructor
        ExCArm();

        // Debug
        void print();

        // Subscribe
        void setMotorEnable(std_msgs::Bool motor_enable);
        void setExCEnable(std_msgs::Bool exc_enable);
        void setEmergencyStop(std_msgs::Bool emergency_stop);
        void setSensorAngle(std_msgs::Float32MultiArray sensor_angle);
        void setTargetAngle(std_msgs::Float32MultiArray target_angle);
        void setTargetPose(geometry_msgs::Pose target_pose);
            void setTargetPoseStart();
            double getDistance();

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> getPose();
            Eigen::Matrix<double, 3, 1> getPosition();
            Eigen::Matrix<double, 3, 1> getEuler();

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> getMidTargetPoseLinearInterpolation();

        // ExC
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobian();
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobianBody();
            Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 6, 6> adjointInverse(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> vector);
        Eigen::Matrix<double, 6, 6> getTransformationMatrix();
            Eigen::Matrix<double, 3, 3> getTransformationEuler();

        // Publish
        bool getMotorEnable();
        bool getExCEnable();
        bool getEmergencyStop();
        std_msgs::Float32MultiArray getMotorAngularVelocity();
            void changeMotorAngularVelocity();
                void setMotorAngularVelocityZero();
                void getMotorAngularVelocityByAngle();
                void getMotorAngularVelocityByExC();


};

// Constructor
ExCArm::ExCArm()
{
    _zero <<
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0;

    // ExC Joint Set Joint
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        exc_joint[i].setJoint(i);
    }
}

// Debug
void ExCArm::print()
{
    std::cout

    << std::endl
    << "pose"
    << std::endl
    << getPose()

    // << std::endl
    // << "mid target pose"
    // << std::endl
    // << getMidTargetPoseLinearInterpolation()

    << std::endl
    << "motor angular velocity"
    << std::endl
    << _motor_angular_velocity

    // << std::endl
    // << "exc jacobian"
    // << std::endl
    // << getExCJacobian()

    << std::endl;
}

// Subscribe
void ExCArm::setMotorEnable(std_msgs::Bool motor_enable)
{
    _motor_enable = motor_enable.data;
}

void ExCArm::setExCEnable(std_msgs::Bool exc_enable)
{
    _exc_enable_old = _exc_enable;
    _exc_enable = exc_enable.data;
}

void ExCArm::setEmergencyStop(std_msgs::Bool emergency_stop)
{
    _emergency_stop = emergency_stop.data;
}

void ExCArm::setSensorAngle(std_msgs::Float32MultiArray sensor_angle)
{
    sensor_angle.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _sensor_angle(i,0) = sensor_angle.data[i];
    }
}

void ExCArm::setTargetAngle(std_msgs::Float32MultiArray target_angle)
{
    target_angle.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _target_angle(i,0) = target_angle.data[i];
    }
}

void ExCArm::setTargetPose(geometry_msgs::Pose target_pose)
{
    if(target_pose != _target_pose_old || _exc_enable != _exc_enable_old)
    {
        _target_pose(0,0) = target_pose.position.x;
        _target_pose(1,0) = target_pose.position.y;
        _target_pose(2,0) = target_pose.position.z;
        _target_pose(3,0) = target_pose.orientation.x;
        _target_pose(4,0) = target_pose.orientation.y;
        _target_pose(5,0) = target_pose.orientation.z;
        setTargetPoseStart();
    }

    _target_pose_old = target_pose;
}

void ExCArm::setTargetPoseStart()
{
    _target_pose_start = _pose;
    _time_start_move = ros::Time::now();
    _duration_time = getDistance()/_linear_velocity;
}

double ExCArm::getDistance()
{
    return sqrt(pow((_target_pose(0,0)-_target_pose_start(0,0)),2)+pow((_target_pose(1,0)-_target_pose_start(1,0)),2)+pow((_target_pose(2,0)-_target_pose_start(2,0)),2));
}

// Forward Kinematics
Eigen::Matrix<double, 6, 1> ExCArm::getPose()
{
    getPosition();
    getEuler();
    _pose(0,0) = _position(0,0);
    _pose(1,0) = _position(1,0);
    _pose(2,0) = _position(2,0);
    _pose(3,0) = _euler(0,0);
    _pose(4,0) = _euler(1,0);
    _pose(5,0) = _euler(2,0);

    return _pose;
}

Eigen::Matrix<double, 3, 1> ExCArm::getPosition()
{
    _position = exc_arm_property.getLink(JOINT_NUMBER);
    for(int i = JOINT_NUMBER-1; 0 <= i; i--)
    {
        _position = exc_arm_property.getRotationMatrix(i, _sensor_angle(i,0))*(exc_arm_property.getLink(i) + _position);
    }

    return _position;
}

Eigen::Matrix<double, 3, 1> ExCArm::getEuler()
{
    _rotation_all << exc_arm_property.getRotationMatrix(0, _sensor_angle(0,0));
    for(int i = 1; i < JOINT_NUMBER; i++)
    {
        _rotation_all = _rotation_all*exc_arm_property.getRotationMatrix(i, _sensor_angle(i,0));
    }

    // ZYX Euler
    _euler(1,0) = -asin(_rotation_all(2,0));
    _euler(0,0) = acos(_rotation_all(0,0)/cos(_euler(1,0)));
    if(_rotation_all(1,0)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    _euler(2,0) = acos(_rotation_all(2,2)/cos(_euler(1,0)));
    if(_rotation_all(2,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);

    for(int i = 0; i < 3; i++)
    {
        if(isnan(_euler(i,0))) _euler(i,0) = 0.0;
    }

    return _euler;
}

// Linear Interpolation
Eigen::Matrix<double, 6, 1> ExCArm::getMidTargetPoseLinearInterpolation()
{
    _midpoint = std::min(std::max((ros::Time::now()-_time_start_move).toSec()/_duration_time, 0.0), 1.0);
    return _midpoint*_target_pose +(1-_midpoint)*_target_pose_start;
}

// Publish
bool ExCArm::getMotorEnable()
{
    return _motor_enable;
}

bool ExCArm::getExCEnable()
{
    return _exc_enable;
}

bool ExCArm::getEmergencyStop()
{
    return _emergency_stop;
}

std_msgs::Float32MultiArray ExCArm::getMotorAngularVelocity()
{
    changeMotorAngularVelocity();

    std_msgs::Float32MultiArray motor_angular_velocity_;
    motor_angular_velocity_.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        motor_angular_velocity_.data[i] = _motor_angular_velocity(i,0);
    }

    return motor_angular_velocity_;
}

void ExCArm::changeMotorAngularVelocity()
{
    if(_emergency_stop)
    {
        setMotorAngularVelocityZero();
        return;
    }

    if(_exc_enable)
    {
        getMotorAngularVelocityByExC();
        return;
    }
    else
    {
        getMotorAngularVelocityByAngle();
        return;
    }
}

void ExCArm::setMotorAngularVelocityZero()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _motor_angular_velocity(i,0) = 0.0;
    }
}

void ExCArm::getMotorAngularVelocityByAngle()
{
    if(_exc_enable) return;

    _motor_angular_velocity = exc_arm_property.getProportionalGainAngleOperating()*(_target_angle - _sensor_angle);
}

void ExCArm::getMotorAngularVelocityByExC()
{
    _motor_angular_velocity = _proportional_gain_exc*(getExCJacobian().inverse())*(getMidTargetPoseLinearInterpolation()-getPose());
}

Eigen::Matrix<double, 6, JOINT_NUMBER> ExCArm::getExCJacobian()
{
    _exc_jacobian = (getTransformationMatrix().inverse())*getExCJacobianBody();

    return _exc_jacobian;
}

Eigen::Matrix<double, 6, JOINT_NUMBER> ExCArm::getExCJacobianBody()
{
    Eigen::Matrix<double, 6, 1> xi_dagger_[JOINT_NUMBER];

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        Eigen::Matrix<double, 4, 4> matrix_;
        matrix_ = exc_arm_property.getGstZero();

        for(int j = JOINT_NUMBER-1; i <= j ; j--)
        {
            matrix_ = exc_joint[j].getExpXiHatTheta(_sensor_angle(j,0))*matrix_;
        }

        xi_dagger_[i] = adjointInverse(matrix_)*exc_joint[i].getXi();

        _exc_jacobian_body(i,0) = xi_dagger_[i](0,0);
        for(int k = 0; k < 6; k++)
        {
            _exc_jacobian_body(k,i) = xi_dagger_[i](k,0);
        }
    }

    return _exc_jacobian_body;
}

Eigen::Matrix<double, 6, 6> ExCArm::adjoint(Eigen::Matrix<double, 4, 4> matrix)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_matrix_;

    rotation_matrix_ <<
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2);

    position_ <<
    matrix(0,3),
    matrix(1,3),
    matrix(2,3);

    adjoint_matrix_ <<
    rotation_matrix_, hat(position_)*rotation_matrix_,
               _zero,                rotation_matrix_;

    return adjoint_matrix_;
}

Eigen::Matrix<double, 6, 6> ExCArm::adjointInverse(Eigen::Matrix<double, 4, 4> matrix)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_inverse_matrix_;

    rotation_matrix_ <<
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2);

    position_ <<
    matrix(0,3),
    matrix(1,3),
    matrix(2,3);

    adjoint_inverse_matrix_ <<
    rotation_matrix_.transpose(), -rotation_matrix_.transpose()*hat(position_),
                           _zero,                 rotation_matrix_.transpose();

    return adjoint_inverse_matrix_;
}

Eigen::Matrix<double, 3, 3> ExCArm::hat(Eigen::Matrix<double, 3, 1> vector)
{
    Eigen::Matrix<double, 3, 3> hat_vector_;

    hat_vector_ <<
             0.0, -vector(2,0),  vector(1,0),
     vector(2,0),          0.0, -vector(0,0),
    -vector(1,0),  vector(0,0),          0.0;

    return hat_vector_;
}

Eigen::Matrix<double, 6, 6> ExCArm::getTransformationMatrix()
{
    getTransformationEuler();

    _transformation_matrix <<
    _rotation_all,                 _zero,
            _zero, _transformation_euler;

    return _transformation_matrix;
}

Eigen::Matrix<double, 3, 3> ExCArm::getTransformationEuler()
{
    _transformation_euler <<
    0.0, -sin(_euler(0,0)), cos(_euler(1,0))*cos(_euler(0,0)),
    0.0,  cos(_euler(0,0)), cos(_euler(1,0))*sin(_euler(0,0)),
    1.0,               0.0,                 -sin(_euler(1,0));

    return _transformation_euler;
}

#endif