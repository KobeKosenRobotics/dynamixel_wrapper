#ifndef CLASS_PID_HPP
#define CLASS_PID_HPP

#include "class_joint.hpp"
#include "class_arm.hpp"

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
 
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
 
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
 
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>
 
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

class Pid
{
    private:
        double _proportional_gain = 1.0, _integral_gain = 0.0, _differential_gain = 1.0;
        Eigen::Matrix<double, 6, 1> _error, _error_old, _error_integral, _error_differential;
        ros::Time _time, _time_old;
        double _dt;
        bool _is_first = true;
        Eigen::Matrix<double, 6, 1> _output;

    public:
        Eigen::Matrix<double, 6, 1> pid(Eigen::Matrix<double, 6, 1> error, ros::Time ros_time_now);
        void resetPid();
};

Eigen::Matrix<double, 6, 1> Pid::pid(Eigen::Matrix<double, 6, 1> error, ros::Time ros_time_now)
{
    if(_is_first)
    {
        _time_old = ros_time_now;
        _error_old = error;
        _error_integral << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        _is_first = false;
        
        _output << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        return _output;
    }

    _time = ros_time_now;
    _dt = (_time - _time_old).toSec();
    _time_old = _time;

    _error = error;
    _error_integral += _dt*_error;
    _error_differential = (1/_dt)*(_error_old-_error);
    
    _output = _proportional_gain*_error + _integral_gain*_error_integral + _differential_gain*_error_differential;
    _error_old = _error;

    // std::cout << _error_integral << std::endl << std::endl << std::endl;
    return _output;
}

void Pid::resetPid()
{
    _is_first = true;
    _error_integral << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

#endif