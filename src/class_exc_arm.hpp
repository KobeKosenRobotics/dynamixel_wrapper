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

        // Inverse Kinematics
        Eigen::Matrix<double, 6, 1> _target_pose;
        double _proportional_gain_exc = 4.0;

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> _target_pose_mid, _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 50;    // _liner_velocity[mm/s]

        // ExC (Exponential Coordinates)

    public:
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

        // Publish
        bool getMotorEnable();
        bool getExCEnable();
        bool getEmergencyStop();
        std_msgs::Float32MultiArray getMotorAngularVelocity();
            void changeMotorAngularVelocity();
                void setMotorAngularVelocityZero();
                void getMotorAngularVelocityByAngle();
                void getMotorAngularVelocityByExC();
                    // Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobian();
                    //     Eigen::<double, 4, 4> adjoint(Eigen::Matrix<double, 4, 4> matrix);
};

// Debug
void ExCArm::print()
{
    std::cout

    << "pose"
    << std::endl
    << getPose()
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
    Eigen::Matrix<double, 3, 3> rotation_all_;
    rotation_all_ << exc_arm_property.getRotationMatrix(0, _sensor_angle(0,0));
    for(int i = 1; i < JOINT_NUMBER; i++)
    {
        rotation_all_ = rotation_all_*exc_arm_property.getRotationMatrix(i, _sensor_angle(i,0));
    }

    // ZYX Euler
    _euler(1,0) = -asin(rotation_all_(2,0));
    _euler(0,0) = acos(rotation_all_(0,0)/cos(_euler(1,0)));
    if(rotation_all_(1,0)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    _euler(2,0) = acos(rotation_all_(2,2)/cos(_euler(1,0)));
    if(rotation_all_(2,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);

    return _euler;
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
    std::cout << "ExC" << std::endl;
}

#endif