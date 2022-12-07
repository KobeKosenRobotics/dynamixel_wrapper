#ifndef ARM_HPP
#define ARM_HPP

#include "class_joint.hpp"
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

class Arm
{
    private:
        const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI;

        // Motor
        dynamixel_wrapper::dynamixel_wrapper_base _dxl_base;
        int _operating_mode = 1;
        Eigen::Matrix<double, 6, 1> _sensor_angle, _sensor_angular_velocity;
        Eigen::Matrix<double, 6, 1> _motor_angle, _motor_angular_velocity;
        Eigen::Matrix<double, 6, 1> _angle_error;
        Eigen::Matrix<double, 6, 1> _target_angle, _target_angle_old;
        double _angle_accuracy = 0.00001;
        double _operating_mode_angular_velocity;

        // Forward Kinematics
        Eigen::Matrix<double, 3, 3> _rotation_all;
        Eigen::Matrix<double, 3, 1> _position, _euler;
        Eigen::Matrix<double, 6, 1> _pose;

        // Inverse Kinematics
        bool _is_first_replace_variables = true;
        double lofx, lofy, lofz, c12, s12,
        l0x, l0y, l0z, t0, c0, s0,
        l1x, l1y, l1z, t1, c1, s1,
        l2x, l2y, l2z, t2, c2, s2,
        l3x, l3y, l3z, t3, c3, s3,
        l4x, l4y, l4z, t4, c4, s4,
        l5x, l5y, l5z, t5, c5, s5;
        double _proportional_gain = 1.0;
        bool _is_first_linear_polation = true;
        double _midpoint, _duration_time, _liniar_velocity = 50;    // _liner_velocity[mm/s]
        Eigen::Matrix<double, 6, 1> _pose_error;
        double _pose_accuracy = 1.0;
        ros::Time _start_time_move;
        Eigen::Matrix<double, 6, 1> _target_pose, _target_pose_old, _target_pose_mid, _target_pose_start, _anguler_velocity;
        Eigen::Matrix<double, 6, 6> _jacobian, _jacobian_inverse;
        Eigen::Matrix<double, 3, 6> _translation_jacobian, _rotation_jacobian;
        Eigen::Matrix<double, 3, 3> _alternating_euler;
        Eigen::Matrix<double, 3, 6> _alternating_rotation;

        // Topic Communication
        geometry_msgs::Pose _msg_old;

    public:
        // Public
        void setTarget(geometry_msgs::Pose msg);
        void update();

        // Initialize
        Arm();
        void initialize();
        Joint joint_offset, joint0, joint1, joint2, joint3, joint4, joint5;
        
        // Debag
        void print();

        // Motor
        Eigen::Matrix<double, 6, 1> getAngle();
        Eigen::Matrix<double, 6, 1> getAngle(geometry_msgs::Pose msg);
        Eigen::Matrix<double, 6, 1> getAngularVelocity();
        void setAngle(Eigen::Matrix<double, 6, 1> angle_rad);
        void setAngularVelocity(Eigen::Matrix<double, 6, 1> angular_velocity_radps);
        void setTargetAngle(Eigen::Matrix<double, 6, 1> target_angle);
        void setTargetAngle(geometry_msgs::Pose target_angle);
        Eigen::Matrix<double, 6, 1> getTargetAngle();
        bool isInTargetAngle();

        // Forward Kinematics
        Eigen::Matrix<double, 3, 1> getPosition();
        Eigen::Matrix<double, 3, 1> getEulerAngle();
        Eigen::Matrix<double, 6, 1> getPose();

        // Inverse Kinematics
        void setTargetPose(Eigen::Matrix<double, 6, 1> target_pose);
        void setTargetPose(geometry_msgs::Pose target_pose);
        Eigen::Matrix<double, 6, 1> linearInterpolation();
        void setStartPose();
        void setStartPose(geometry_msgs::Pose msg);
        double getDistance();
        bool isInTargetPose();
        Eigen::Matrix<double, 6, 1> inverseKinematics();
        Eigen::Matrix<double, 6, 6> getJacobian();
        Eigen::Matrix<double, 3, 6> getTranslationJacobian();
        Eigen::Matrix<double, 3, 6> getRotationJacobian();
        void replaceVariables();

        // Simulation
        void simulationUpdate();
        void tf_broadcaster();
};

// Public
void Arm::setTarget(geometry_msgs::Pose msg)
{
    _operating_mode_angular_velocity = msg.orientation.w;
    if(_operating_mode_angular_velocity < 0.5) setTargetAngle(msg);
    else setTargetPose(msg);
    setStartPose(msg);
}

void Arm::update()
{
    simulationUpdate();
    getPose();
    print();

    if(_operating_mode_angular_velocity < 0.5) setAngle(getTargetAngle());
    else setAngularVelocity(inverseKinematics());
}

// Initialize
Arm::Arm()
{
    initialize();
}

void Arm::initialize()
{
    #ifndef SIMULATION
    // std::string _port_name("/dev/ttyUSB0");
    // int _baudrate = 1000000;
    // _dxl_base.initialize(_port_name, _baudrate);
    #endif

    joint_offset.initialize(  0.0, 0.0, 159.0, 'o');
          joint0.initialize(  0.0, 0.0,   0.0, 'z', 0.0,   0.0, 1, _dxl_base, dynamixel_wrapper::PH54_200_S500_R, _operating_mode, 1.0);
          joint1.initialize( 30.0, 0.0, 264.0, 'y', 0.0, -36.3, 2, _dxl_base, dynamixel_wrapper::H54_200_S500_R,  _operating_mode, 0.0);
          joint2.initialize(-30.0, 0.0, 258.0, 'y', 0.0, -45.0, 3, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  _operating_mode, 0.0);
          joint3.initialize(  0.0, 0.0,   0.0, 'z', 0.0,   0.0, 4, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  _operating_mode, 0.0);
          joint4.initialize(  0.0, 0.0, 123.0, 'y', 0.0,   0.0, 5, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  _operating_mode, 0.0);
          joint5.initialize(  0.0, 0.0,   0.0, 'z', 0.0,   0.0, 6, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  _operating_mode, 0.0);
}

// Debag
void Arm::print()
{
    std::cout
    << "pose"
    << std::endl
    << _pose
    << std::endl

    // << std::endl
    // << "jacobian determinant"
    // << std::endl
    // << _jacobian.determinant()
    // << std::endl

    // << std::endl
    // << "alternating euler determinant"
    // << std::endl
    // << _alternating_euler.determinant()
    // << std::endl

    // << std::endl
    // << "angular velocity"
    // << std::endl
    // << inverseKinematics()
    // << std::endl

    // << std::endl
    // << "eye"
    // << std::endl
    // << _alternating_euler.inverse()*_alternating_euler
    // << std::endl

    << std::endl
    << "operating mode angular velocity"
    << std::endl
    << _operating_mode_angular_velocity
    << std::endl

    << std::endl
    << "is in target pose"
    << std::endl
    << isInTargetPose()
    << std::endl

    << std::endl
    << "pose error"
    << std::endl
    << _pose_error
    << std::endl

    << std::endl
    << "is in target angle"
    << std::endl
    << isInTargetAngle()
    << std::endl

    << std::endl
    << "angle error"
    << std::endl
    << _angle_error
    << std::endl

    << std::endl
    << "midpoint"
    << std::endl
    << _midpoint
    << std::endl

    << std::endl;
}

// Motor
Eigen::Matrix<double, 6, 1> Arm::getAngle()
{
    _sensor_angle(0,0) = joint0.getPresentPosition();
    _sensor_angle(1,0) = joint1.getPresentPosition();
    _sensor_angle(2,0) = joint2.getPresentPosition();
    _sensor_angle(3,0) = joint3.getPresentPosition();
    _sensor_angle(4,0) = joint4.getPresentPosition();
    _sensor_angle(5,0) = joint5.getPresentPosition();
    _angle_error = _target_angle-_sensor_angle;
    return _sensor_angle;
}

Eigen::Matrix<double, 6, 1> Arm::getAngle(geometry_msgs::Pose msg)
{
    _sensor_angle(0,0) = joint0.getPresentPosition(msg.position.x);
    _sensor_angle(1,0) = joint1.getPresentPosition(msg.position.y);
    _sensor_angle(2,0) = joint2.getPresentPosition(msg.position.z);
    _sensor_angle(3,0) = joint3.getPresentPosition(msg.orientation.x);
    _sensor_angle(4,0) = joint4.getPresentPosition(msg.orientation.y);
    _sensor_angle(5,0) = joint5.getPresentPosition(msg.orientation.z);
    _angle_error = _target_angle-_sensor_angle;
    return _sensor_angle;
}

Eigen::Matrix<double, 6, 1> Arm::getAngularVelocity()
{
    _sensor_angular_velocity(0,0) = joint0.getPresentVelocity();
    _sensor_angular_velocity(1,0) = joint1.getPresentVelocity();
    _sensor_angular_velocity(2,0) = joint2.getPresentVelocity();
    _sensor_angular_velocity(3,0) = joint3.getPresentVelocity();
    _sensor_angular_velocity(4,0) = joint4.getPresentVelocity();
    _sensor_angular_velocity(5,0) = joint5.getPresentVelocity();
    return _sensor_angular_velocity;
}

void Arm::setAngle(Eigen::Matrix<double, 6, 1> angle_rad)
{
    _motor_angle = angle_rad;
    joint0.setGoalPosition(_motor_angle(0,0));
    joint1.setGoalPosition(_motor_angle(1,0));
    joint2.setGoalPosition(_motor_angle(2,0));
    joint3.setGoalPosition(_motor_angle(3,0));
    joint4.setGoalPosition(_motor_angle(4,0));
    joint5.setGoalPosition(_motor_angle(5,0));
}

void Arm::setAngularVelocity(Eigen::Matrix<double, 6, 1> angular_velocity_radps)
{
    _motor_angular_velocity = angular_velocity_radps;
    joint0.setGOalVelocity(_motor_angular_velocity(0,0));
    joint1.setGOalVelocity(_motor_angular_velocity(1,0));
    joint2.setGOalVelocity(_motor_angular_velocity(2,0));
    joint3.setGOalVelocity(_motor_angular_velocity(3,0));
    joint4.setGOalVelocity(_motor_angular_velocity(4,0));
    joint5.setGOalVelocity(_motor_angular_velocity(5,0));
}

void Arm::setTargetAngle(Eigen::Matrix<double, 6, 1> target_angle)
{
    _target_angle = target_angle;
}

void Arm::setTargetAngle(geometry_msgs::Pose target_angle)
{
    _target_angle(0,0) = target_angle.position.x;
    _target_angle(1,0) = target_angle.position.y;
    _target_angle(2,0) = target_angle.position.z;
    _target_angle(3,0) = target_angle.orientation.x;
    _target_angle(4,0) = target_angle.orientation.y;
    _target_angle(5,0) = target_angle.orientation.z;
}

Eigen::Matrix<double, 6, 1> Arm::getTargetAngle()
{
    return _target_angle;
}

bool Arm::isInTargetAngle()
{
    if(_angle_error.transpose()*_angle_error < _angle_accuracy) return true;
    return false;
}

// Forward Kinematics
Eigen::Matrix<double, 3, 1> Arm::getPosition()
{
    _position = joint_offset.getRotationMatrix()*(joint_offset.getLink()+joint0.getRotationMatrix()*(joint0.getLink()+joint1.getRotationMatrix()*(joint1.getLink()+joint2.getRotationMatrix()*(joint2.getLink()+joint3.getRotationMatrix()*(joint3.getLink()+joint4.getRotationMatrix()*(joint4.getLink()+joint5.getRotationMatrix()*(joint5.getLink())))))));
    return _position;
}

Eigen::Matrix<double, 3, 1> Arm::getEulerAngle()
{
    _rotation_all = joint0.getRotationMatrix()*joint1.getRotationMatrix()*joint2.getRotationMatrix()*joint3.getRotationMatrix()*joint4.getRotationMatrix()*joint5.getRotationMatrix();

    // ZYX Euler
    _euler(1,0) = -asin(_rotation_all(2,0));
    _euler(0,0) = acos(_rotation_all(0,0)/cos(_euler(1,0)));
    if(_rotation_all(1,0)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    _euler(2,0) = acos(_rotation_all(2,2)/cos(_euler(1,0)));
    if(_rotation_all(2,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);
    return _euler;
}

Eigen::Matrix<double, 6, 1> Arm::getPose()
{
    getPosition();
    getEulerAngle();
    _pose(0,0) = _position(0,0);
    _pose(1,0) = _position(1,0);
    _pose(2,0) = _position(2,0);
    _pose(3,0) = _euler(0,0);
    _pose(4,0) = _euler(1,0);
    _pose(5,0) = _euler(2,0);
    _pose_error = _target_pose-_pose;
    return _pose;
}

// Inverse Kinematics
void Arm::setTargetPose(Eigen::Matrix<double, 6, 1> target_pose)
{
    _target_pose = target_pose;
    setStartPose();
}

void Arm::setTargetPose(geometry_msgs::Pose target_pose)
{
    _target_pose(0,0) = target_pose.position.x;
    _target_pose(1,0) = target_pose.position.y;
    _target_pose(2,0) = target_pose.position.z;
    _target_pose(3,0) = target_pose.orientation.z;
    _target_pose(4,0) = target_pose.orientation.y;
    _target_pose(5,0) = target_pose.orientation.x;
    setStartPose();
}

Eigen::Matrix<double, 6, 1> Arm::linearInterpolation()
{
    _midpoint = std::min(std::max((ros::Time::now()-_start_time_move).toSec()/_duration_time, 0.0), 1.0);
    _target_pose_mid = _midpoint*_target_pose +(1-_midpoint)*_target_pose_start;
    return _target_pose_mid;
}

void Arm::setStartPose()
{
    if(_target_pose != _target_pose_old)
    {
        _target_pose_start = _pose;
        _start_time_move = ros::Time::now();
        _duration_time = getDistance()/_liniar_velocity;
    }
    _target_pose_old = _target_pose;
}

void Arm::setStartPose(geometry_msgs::Pose msg)
{
    if(msg != _msg_old)
    {
        _target_pose_start = _pose;
        _start_time_move = ros::Time::now();
        _duration_time = getDistance()/_liniar_velocity;
    }
    _msg_old = msg;
}

double Arm::getDistance()
{
    return sqrt(pow((_target_pose(0,0)-_target_pose_start(0,0)),2)+pow((_target_pose(1,0)-_target_pose_start(1,0)),2)+pow((_target_pose(2,0)-_target_pose_start(2,0)),2));
}

bool Arm::isInTargetPose()
{
    if(_pose_error.transpose()*_pose_error < _pose_accuracy) return true;
    return false;
}

Eigen::Matrix<double, 6, 1> Arm::inverseKinematics()
{
    _jacobian = getJacobian();
    _jacobian_inverse = _jacobian.inverse();
    _anguler_velocity = _proportional_gain*_jacobian_inverse*(linearInterpolation()-_pose);
    return _anguler_velocity;
}

Eigen::Matrix<double, 6, 6> Arm::getJacobian()
{
    replaceVariables();
    
    _translation_jacobian = getTranslationJacobian();
    _rotation_jacobian = getRotationJacobian();

    _jacobian <<
    _translation_jacobian(0,0), _translation_jacobian(0,1), _translation_jacobian(0,2), _translation_jacobian(0,3), _translation_jacobian(0,4), _translation_jacobian(0,5),
    _translation_jacobian(1,0), _translation_jacobian(1,1), _translation_jacobian(1,2), _translation_jacobian(1,3), _translation_jacobian(1,4), _translation_jacobian(1,5),
    _translation_jacobian(2,0), _translation_jacobian(2,1), _translation_jacobian(2,2), _translation_jacobian(2,3), _translation_jacobian(2,4), _translation_jacobian(2,5),
    _rotation_jacobian(0,0)   , _rotation_jacobian(0,1)   , _rotation_jacobian(0,2)   , _rotation_jacobian(0,3)   , _rotation_jacobian(0,4)   , _rotation_jacobian(0,5)   ,
    _rotation_jacobian(1,0)   , _rotation_jacobian(1,1)   , _rotation_jacobian(1,2)   , _rotation_jacobian(1,3)   , _rotation_jacobian(1,4)   , _rotation_jacobian(1,5)   ,
    _rotation_jacobian(2,0)   , _rotation_jacobian(2,1)   , _rotation_jacobian(2,2)   , _rotation_jacobian(2,3)   , _rotation_jacobian(2,4)   , _rotation_jacobian(2,5)   ;

    return _jacobian;
}

Eigen::Matrix<double, 3, 6> Arm::getTranslationJacobian()
{
    _translation_jacobian(0,0) = - s0*(l0x + c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) + s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))))) - c0*(l0y + l1y + l2y + c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(0,1) = c0*(c1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(0,2) = c0*(c1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(0,3) = s0*(s3*(l3y + l4y + l5y*c5 + l5x*s5) - c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))) - c12*c0*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(0,4) = s0*s3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)) - c0*(c1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) + s1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))));
    _translation_jacobian(0,5) = - c0*(c1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5)) - s1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5))) - s0*(c3*(l5x*c5 - l5y*s5) - c4*s3*(l5y*c5 + l5x*s5));

    _translation_jacobian(1,0) = c0*(l0x + c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) + s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))))) - s0*(l0y + l1y + l2y + c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(1,1) = s0*(c1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(1,2) = s0*(c1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(1,3) = - c0*(s3*(l3y + l4y + l5y*c5 + l5x*s5) - c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))) - c12*s0*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(1,4) = - s0*(c1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) + s1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)))) - c0*s3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z));
    _translation_jacobian(1,5) = c0*(c3*(l5x*c5 - l5y*s5) - c4*s3*(l5y*c5 + l5x*s5)) - s0*(c1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5)) - s1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5)));

    _translation_jacobian(2,0) = 0.0;
    _translation_jacobian(2,1) = - c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))));
    _translation_jacobian(2,2) = - c1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))));
    _translation_jacobian(2,3) = s12*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(2,4) = s1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) - c1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)));
    _translation_jacobian(2,5) = c1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5)) + s1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5));

    return _translation_jacobian;
}

Eigen::Matrix<double, 3, 6> Arm::getRotationJacobian()
{
    _alternating_euler <<
    0.0, -sin(_euler(0,0)), cos(_euler(1,0))*cos(_euler(0,0)),
    0.0,  cos(_euler(0,0)), cos(_euler(1,0))*sin(_euler(0,0)),
    1.0,               0.0,                 -sin(_euler(1,0));

    _alternating_rotation <<
    0.0, -s0, -s0, s12*c1,  c0*s1*s2*s3 -c0*c1*c2*s3 - c3*s0,    c0*c1*c4*s2 - s0*s3*s4 +c0*c2*c4*s1 +c0*c1*c2*c3*s4 -c0*c3*s1*s2*s4,
    0.0,  c0,  c0, s12*s0, c0*c3 - c1*c2*s0*s3 + s0*s1*s2*s3, c0*s3*s4 + c1*c4*s0*s2 + c2*c4*s0*s1 + c1*c2*c3*s0*s4 - c3*s0*s1*s2*s4,
    1.0, 0.0, 0.0,    c12,                            s12*s3,                        c1*c2*c4 - c4*s1*s2 - c1*c3*s2*s4 - c2*c3*s1*s4;

    _rotation_jacobian = _alternating_euler.inverse()*_alternating_rotation;
    return _rotation_jacobian;
}

void Arm::replaceVariables()
{
    if(_is_first_replace_variables)
    {
        lofx = joint_offset.getLink('x');
        lofy = joint_offset.getLink('y');
        lofz = joint_offset.getLink('z');

        l0x = joint0.getLink('x');
        l0y = joint0.getLink('y');
        l0z = joint0.getLink('z');
        
        l1x = joint1.getLink('x');
        l1y = joint1.getLink('y');
        l1z = joint1.getLink('z');
        
        l2x = joint2.getLink('x');
        l2y = joint2.getLink('y');
        l2z = joint2.getLink('z');
        
        l3x = joint3.getLink('x');
        l3y = joint3.getLink('y');
        l3z = joint3.getLink('z');
        
        l4x = joint4.getLink('x');
        l4y = joint4.getLink('y');
        l4z = joint4.getLink('z');
        
        l5x = joint5.getLink('x');
        l5y = joint5.getLink('y');
        l5z = joint5.getLink('z');

        _is_first_replace_variables = false;
    }

    t0 = joint0.getGlobalAngle();
    t1 = joint1.getGlobalAngle();
    t2 = joint2.getGlobalAngle();
    t3 = joint3.getGlobalAngle();
    t4 = joint4.getGlobalAngle();
    t5 = joint5.getGlobalAngle();

    c0 = cos(t0);
    c1 = cos(t1);
    c2 = cos(t2);
    c3 = cos(t3);
    c4 = cos(t4);
    c5 = cos(t5);
    c12 = cos(t1+t2);

    s0 = sin(t0);
    s1 = sin(t1);
    s2 = sin(t2);
    s3 = sin(t3);
    s4 = sin(t4);
    s5 = sin(t5);
    s12 = sin(t1+t2);
}

// Simulation
void Arm::simulationUpdate()
{
    joint0.simulationUpdate();
    joint1.simulationUpdate();
    joint2.simulationUpdate();
    joint3.simulationUpdate();
    joint4.simulationUpdate();
    joint5.simulationUpdate();
    tf_broadcaster();
}

void Arm::tf_broadcaster()
{
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // XYZ Euler
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "euler";
    transformStamped.transform.translation.x = _position(0,0)/1000.0;
    transformStamped.transform.translation.y = _position(1,0)/1000.0;
    transformStamped.transform.translation.z = _position(2,0)/1000.0;
    
    q.setRPY(_euler(2,0), _euler(1,0), _euler(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
    
    // Joint 0
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "joint0";
    transformStamped.transform.translation.x = joint_offset.simulationLink('x');
    transformStamped.transform.translation.y = joint_offset.simulationLink('y');
    transformStamped.transform.translation.z = joint_offset.simulationLink('z');
    
    q.setRPY(joint0.simulationAngle('x'), joint0.simulationAngle('y'), joint0.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
    
    // Joint 1
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint0";
    transformStamped.child_frame_id = "joint1";
    transformStamped.transform.translation.x = joint0.simulationLink('x');
    transformStamped.transform.translation.y = joint0.simulationLink('y');
    transformStamped.transform.translation.z = joint0.simulationLink('z');
    
    q.setRPY(joint1.simulationAngle('x'), joint1.simulationAngle('y'), joint1.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 2
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint1";
    transformStamped.child_frame_id = "joint2";
    transformStamped.transform.translation.x = joint1.simulationLink('x');
    transformStamped.transform.translation.y = joint1.simulationLink('y');
    transformStamped.transform.translation.z = joint1.simulationLink('z');
    
    q.setRPY(joint2.simulationAngle('x'), joint2.simulationAngle('y'), joint2.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 3
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint2";
    transformStamped.child_frame_id = "joint3";
    transformStamped.transform.translation.x = joint2.simulationLink('x');
    transformStamped.transform.translation.y = joint2.simulationLink('y');
    transformStamped.transform.translation.z = joint2.simulationLink('z');
    
    q.setRPY(joint3.simulationAngle('x'), joint3.simulationAngle('y'), joint3.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 4
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint3";
    transformStamped.child_frame_id = "joint4";
    transformStamped.transform.translation.x = joint3.simulationLink('x');
    transformStamped.transform.translation.y = joint3.simulationLink('y');
    transformStamped.transform.translation.z = joint3.simulationLink('z');
    
    q.setRPY(joint4.simulationAngle('x'), joint4.simulationAngle('y'), joint4.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 5
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint4";
    transformStamped.child_frame_id = "joint5";
    transformStamped.transform.translation.x = joint4.simulationLink('x');
    transformStamped.transform.translation.y = joint4.simulationLink('y');
    transformStamped.transform.translation.z = joint4.simulationLink('z');
    
    q.setRPY(joint5.simulationAngle('x'), joint5.simulationAngle('y'), joint5.simulationAngle('z'));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}
#endif