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
        char _axis;
        Eigen::Matrix<double, 3, 1> _link;
        dynamixel_wrapper::dynamixel_wrapper _motor;
        double _homing_offset;

        bool _is_valid, _is_valid_old = true;

        double _sensor_angle, _sensor_angular_velocity;
        double _simulation_angle, _simulation_angular_velocity;
        double _motor_angle, _motor_angular_velocity;
    public:
        // Initialize Joint
        Joint();
        void initialize(double link_x, double link_y, double link_z, char axis);
        void initialize(double link_x, double link_y, double link_z, char axis, double homing_offset, int id, dynamixel_wrapper::dynamixel_wrapper_base dxl_base, dynamixel_wrapper::dynamixel_wrapper_config motor_config, int operating_mode);
        void setHomingOffset(double homing_offset);
        
        void setTorqueEnable(bool state);
        void setLED(int red, int green, int blue);

        double getPresentPosition();
        double getPresentVelocity();
        void setGoalPosition(double motor_angle_rad);
        void setGOalVelocity(double motor_angular_velocity_radps);

        void print();
        void update();
        Eigen::Matrix<double, 3, 3> getRotationMatrix();

        // Simulation
        Eigen::Matrix<double, 3, 1> simulationLink();
        double simulationLink(char axis);
        double simulationAngle(char axis);
        double simulationAngle();
        double simulationAngularVelocity(char axis);
};

Joint::Joint(){}

void Joint::initialize(double link_x, double link_y, double link_z, char axis)
{
    _axis = axis;
    _link << link_x, link_y, link_z;
}
void Joint::initialize(double link_x, double link_y, double link_z, char axis, double homing_offset, int id, dynamixel_wrapper::dynamixel_wrapper_base dxl_base, dynamixel_wrapper::dynamixel_wrapper_config motor_config, int operating_mode)
{
    initialize(link_x, link_y, link_z, axis);
    #ifndef SIMULATION
    _motor.initialize(id, dxl_base, motor_config, operating_mode);
    #endif
    setTorqueEnable(false);
    setHomingOffset(homing_offset);

}

void Joint::setTorqueEnable(bool state)
{
    #ifndef SIMULATION
    _motor.setTorqueEnable(state);
    #endif
}

void Joint::setHomingOffset(double homing_offset)
{
    setTorqueEnable(false);
    _homing_offset = homing_offset;
    #ifndef SIMULATION
    _motor.setHomingOffset(_homing_offset*rad2deg);
    #endif
}

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
        #ifndef SIMULATION
        _motor.setGoalPosition(_motor_angle*rad2deg);
        #endif
        #ifdef SIMULATION
        _simulation_angle = _motor_angle;
        #endif
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
        _motor.setGoalPosition(_motor_angular_velocity*radps2rpm);
        #endif
        #ifdef SIMULATION
        _simulation_angular_velocity = _motor_angular_velocity;
        #endif
    }
}

void Joint::print()
{
    std::cout << _axis << std::endl << std::endl;
    std::cout << _link << std::endl;
}

// Simulation
Eigen::Matrix<double, 3, 1> Joint::simulationLink()
{
    return _link/1000.0;
}
double Joint::simulationLink(char axis)
{
    if(axis == 'x') return _link(0,0)/1000.0;
    if(axis == 'y') return _link(1,0)/1000.0;
    if(axis == 'z') return _link(2,0)/1000.0;
    else
    {
        std::cout << "error: axis must be x, y or z" << std::endl;
        return 0.0;
    }
}
double Joint::simulationAngle(char axis)
{
    if(axis == _axis) return _simulation_angle-_homing_offset;
    else return 0.0;
}
double Joint::simulationAngle()
{
    return _simulation_angle;//-_homing_offset;
}
double Joint::simulationAngularVelocity(char axis)
{
    if(axis == _axis) return _simulation_angular_velocity;
    else return 0.0;
}
#endif