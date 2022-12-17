#ifndef CLASS_EXC_ARM_HPP
#define CLASS_EXC_ARM_HPP

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
        Eigen::Matrix<double, 3, 3> _rotation_all;

        // Inverse Kinematics
        Eigen::Matrix<double, 6, 1> _target_pose;
        double _proportional_gain = 4.0;

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> _target_pose_mid, _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 50;    // _liner_velocity[mm/s]

        // ExC (Exponential Coordinates)

    public:
        // Subscribe
        void setMotorEnable(std_msgs::Bool motor_enable);
        void setExCEnable(std_msgs::Bool exc_enable);
        void setEmergencyStop(std_msgs::Bool emergency_stop);
        void setTargetAngle(std_msgs::Float32MultiArray target_angle);
        void setTargetPose(geometry_msgs::Pose target_pose);
            void setTargetPoseStart();
            double getDistance();

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> getPose();
            Eigen::Matrix<double, 3, 1> getPosition();
            Eigen::Matrix<double, 3, 1> getEuler();
};


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

void ExCArm::setTargetAngle(std_msgs::Float32MultiArray target_angle)
{
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

void ExCArm::getDistance()
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
    _pose << 0.0, 0.0, 0.0;
    for(int i = JOINT_NUMBER; i > 0; i--)
    {
        _pose = ()
    }
}
Eigen::Matrix<double, 3, 1> ExCArm::getEuler();

#endif