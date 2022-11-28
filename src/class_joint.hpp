#ifndef JOINT_HPP
#define JOINT_HPP

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

class Joint
{
    private:
        const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

        // Initialize
        char _axis;
        Eigen::Matrix<double, 3, 1> _link;
        dynamixel_wrapper::dynamixel_wrapper _motor;
        int _operating_mode;
        double _homing_offset;

        // Motor
        bool _is_valid, _is_valid_old = true;
        double _sensor_angle, _sensor_angular_velocity;
        double _simulation_angle, _simulation_angular_velocity;
        double _global_theta;
        double _motor_angle, _motor_angular_velocity;
        double _proportional_gain = 2.0;

        // Rotation Matrix
        Eigen::Matrix<double, 3, 3> _rotation_matrix;

        // Simulation
        bool _is_first_simulation = true;
        ros::Time _start_time_simulation, _end_time_simulation;
    public:
        // Initialize
        Joint();
        void initialize(double link_x, double link_y, double link_z, char axis);
        void initialize(double link_x, double link_y, double link_z, char axis, double homing_offset, double adjust_deg, int id, dynamixel_wrapper::dynamixel_wrapper_base dxl_base, dynamixel_wrapper::dynamixel_wrapper_config motor_config, int operating_mode);

        void setHomingOffset(double homing_offset, double adjust_deg);
        void setTorqueEnable(bool state);

        // Motor
        void setLED(int red, int green, int blue);
        double getPresentPosition();
        double getPresentVelocity();
        void setGoalPosition(double motor_angle_rad);
        void setGOalVelocity(double motor_angular_velocity_radps);

        // Rotation Matrix
        double getGlobalAngle();
        Eigen::Matrix<double, 3, 3> getRotationMatrix();
        Eigen::Matrix<double, 3, 1> getLink();
        double getLink(char axis);

        // Simulation
        void simulationUpdate();
        double simulationLink(char axis);
        double simulationAngle(char axis);
};


// Initialize
Joint::Joint(){}

void Joint::initialize(double link_x, double link_y, double link_z, char axis)
{
    _axis = axis;
    _link << link_x, link_y, link_z;
}

void Joint::initialize(double link_x, double link_y, double link_z, char axis, double homing_offset, double adjust_deg, int id, dynamixel_wrapper::dynamixel_wrapper_base dxl_base, dynamixel_wrapper::dynamixel_wrapper_config motor_config, int operating_mode)
{
    initialize(link_x, link_y, link_z, axis);
    _operating_mode = operating_mode;
    #ifndef SIMULATION
    _motor.initialize(id, dxl_base, motor_config, operating_mode);
    #endif
    setTorqueEnable(false);
    setHomingOffset(homing_offset, adjust_deg);
    _is_first_simulation = true;
}

void Joint::setTorqueEnable(bool state)
{
    #ifndef SIMULATION
    _motor.setTorqueEnable(state);
    #endif
}

void Joint::setHomingOffset(double homing_offset, double adjust_deg)
{
    setTorqueEnable(false);
    _homing_offset = homing_offset;
    #ifndef SIMULATION
    _motor.setHomingOffset(-_homing_offset*rad2deg-adjust_deg);
    #endif
}

// Motor
void Joint::setLED(int red, int green, int blue)
{
    #ifndef SIMULATION
    _motor.setLED(red, green, blue);
    #endif
}

double Joint::getPresentPosition()
{
    #ifndef SIMULATION
    _sensor_angle = _motor.getPresentPosition()*deg2rad;
    #endif
    #ifdef SIMULATION
    _sensor_angle = _simulation_angle;
    #endif
    return _sensor_angle;
}

double Joint::getPresentVelocity()
{
    #ifndef SIMULATION
    _sensor_angular_velocity = _motor.getPresentVelocity()*rpm2radps;
    #endif
    #ifdef SIMULATION
    _sensor_angular_velocity = _simulation_angular_velocity;
    #endif
    return _sensor_angular_velocity;
}

void Joint::setGoalPosition(double motor_angle_rad)
{
    if(isnan(motor_angle_rad))
    {
        _is_valid = false;
        if(_is_valid != _is_valid_old)
        {
            setLED(255, 0, 0);
        }
        _is_valid_old = _is_valid;
        return;
    }
    else
    {
        _motor_angle = motor_angle_rad;

        if(_operating_mode == 1)
        {
            #ifndef SIMULATION
            _motor.setGoalVelocity(_proportional_gain*(_motor_angle-_sensor_angle));
            #endif
            _simulation_angular_velocity = _proportional_gain*(_motor_angle-_sensor_angle);
        }
        if(_operating_mode == 4)
        {
            
            #ifndef SIMULATION
            _motor.setGoalPosition(_motor_angle*rad2deg);
            #endif
            #ifdef SIMULATION
            _simulation_angle = _motor_angle;
            #endif
        }
    }
}

void Joint::setGOalVelocity(double motor_angular_velocity_radps)
{
    if(isnan(motor_angular_velocity_radps))
    {
        _is_valid = false;
        if(_is_valid != _is_valid_old)
        {
            setLED(255, 0, 0);
        }
        _is_valid_old = _is_valid;
        return;
    }
    else
    {
        _motor_angular_velocity = motor_angular_velocity_radps;
        #ifndef SIMULATION
        _motor.setGoalVelocity(_motor_angular_velocity*radps2rpm);
        #endif
        #ifdef SIMULATION
        _simulation_angular_velocity = _motor_angular_velocity;
        #endif
    }
}

// Rotation Matrix
double Joint::getGlobalAngle()
{
    return _sensor_angle+_homing_offset;
}

Eigen::Matrix<double, 3, 3> Joint::getRotationMatrix()
{
    if(_axis == 'x')
    {
        _rotation_matrix << 1.0,                   0.0,                    0.0, 
                            0.0, cos(getGlobalAngle()), -sin(getGlobalAngle()),
                            0.0, sin(getGlobalAngle()),  cos(getGlobalAngle());
    }
    else if(_axis == 'y')
    {
        _rotation_matrix <<  cos(getGlobalAngle()), 0.0, sin(getGlobalAngle()),
                                               0.0, 1.0,                   0.0,
                            -sin(getGlobalAngle()), 0.0, cos(getGlobalAngle());
    }
    else if(_axis == 'z')
    {
        _rotation_matrix << cos(getGlobalAngle()), -sin(getGlobalAngle()), 0.0,
                            sin(getGlobalAngle()),  cos(getGlobalAngle()), 0.0,
                                              0.0,                    0.0, 1.0;
    }
    else
    {
        _rotation_matrix << 1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0;
    }
    return _rotation_matrix;
}

Eigen::Matrix<double, 3, 1> Joint::getLink()
{
    return _link;
}

double Joint::getLink(char axis)
{
    if(axis == 'x') return _link(0,0);
    else if(axis == 'y') return _link(1,0);
    else if(axis == 'z') return _link(2,0);

    std::cout << "error: axis must be x, y or z" << std::endl;
    return 0.0;
}

// Simulation
void Joint::simulationUpdate()
{
    if(_operating_mode == 1)
    {
        if(_is_first_simulation)
        {
            _is_first_simulation = false;
            _start_time_simulation = ros::Time::now();
        }
        _end_time_simulation = ros::Time::now();
        _simulation_angle += (_end_time_simulation-_start_time_simulation).toSec()*_simulation_angular_velocity;
        _start_time_simulation = _end_time_simulation;
    }
}

double Joint::simulationLink(char axis)
{
    return getLink(axis)/1000.0;
}

double Joint::simulationAngle(char axis)
{
    if(axis == _axis) return getGlobalAngle();
    return 0.0;
}

#endif