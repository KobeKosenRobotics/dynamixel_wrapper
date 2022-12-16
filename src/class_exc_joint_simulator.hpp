#ifndef CLASS_EXC_JOINT_SIMULATOR_HPP
#define CLASS_EXC_JOINT_SIMULATOR_HPP

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

class ExCJointSimulator
{
    private:
        bool _is_first_simulation = true;
        ros::Time _start_time_simulation, _end_time_simulation;
        double _simulation_angular_velocity, _simulation_angle = 0.0;
        int _joint, _axis;

    public:
        void initialize(int joint);

        void angularVelocity2Angle(double angular_velocity);
        double getSimulationAngle();
        double getSimulationAngle(int axis);
};

void ExCJointSimulator::initialize(int joint)
{
    _joint = joint;
    _axis = exc_arm_property.getAxis(_joint);
}

void ExCJointSimulator::angularVelocity2Angle(double angular_velocity)
{
    _simulation_angular_velocity = angular_velocity;

    if(_is_first_simulation)
    {
        _is_first_simulation = false;
        _start_time_simulation = ros::Time::now();
    }
    _end_time_simulation = ros::Time::now();
    _simulation_angle += (_end_time_simulation-_start_time_simulation).toSec()*_simulation_angular_velocity;
    _start_time_simulation = _end_time_simulation;
}

double ExCJointSimulator::getSimulationAngle()
{
    return _simulation_angle;
}

double ExCJointSimulator::getSimulationAngle(int axis)
{
    if(axis == _axis) return _simulation_angle;
    else return 0.0;
}

#endif