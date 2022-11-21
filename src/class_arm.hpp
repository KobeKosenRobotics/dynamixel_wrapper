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

        // Forward Kinematics
        Eigen::Matrix<double, 3, 3> _rotation_all;
        Eigen::Matrix<double, 3, 1> _position, _euler;
        Eigen::Matrix<double, 6, 1> _pose;

        // Inverse Kinematics
        double d, l1, l2, l3, l4, l5, l6, c1, c2, c3, c4, c5, c6, c23, s1, s2, s3, s4, s5, s6, s23;
        double _proportional_gain = 0.01;
        bool _is_first_linear_polation = true;
        double _midpoint, _duration_time, _liniar_velocity = 50;    // _liner_velocity[mm/s]
        ros::Time _start_time_move;
        Eigen::Matrix<double, 6, 1> _target_pose, _target_pose_old, _target_pose_mid, _target_pose_start, _pose_offset, _anguler_velocity;
        Eigen::Matrix<double, 6, 6> _jacobian, _jacobian_inverse;
        Eigen::Matrix<double, 3, 6> _translation_jacobian, _rotation_jacobian;
        Eigen::Matrix<double, 3, 3> _alternating_euler;
        Eigen::Matrix<double, 3, 6> _alternating_rotation;

    public:
        // Initialize
        Arm();
        void initialize();
        Joint joint_offset, joint0, joint1, joint2, joint3, joint4, joint5;
        
        // Debag
        void print();

        // Motor
        Eigen::Matrix<double, 6, 1> getAngle();
        Eigen::Matrix<double, 6, 1> getAngularVelocity();
        void setAngle(Eigen::Matrix<double, 6, 1> angle_rad);
        void setAngularVelocity(Eigen::Matrix<double, 6, 1> angular_velocity_radps);

        // Forward Kinematics
        Eigen::Matrix<double, 3, 1> getPosition();
        Eigen::Matrix<double, 3, 1> getEulerAngle();
        Eigen::Matrix<double, 6, 1> getPose();

        // Inverse Kinematics
        void setTargetPose(Eigen::Matrix<double, 6, 1> target_pose);
        void setTargetPose(geometry_msgs::Pose target_pose);
        Eigen::Matrix<double, 6, 1> linearInterpolation();
        void setStartPose();
        double getDistance();
        Eigen::Matrix<double, 6, 1> inverseKinematics();
        Eigen::Matrix<double, 6, 6> getJacobian();
        Eigen::Matrix<double, 3, 6> getTranslationJacobian();
        Eigen::Matrix<double, 3, 6> getRotationJacobian();
        void replaceVariables();

        // Simulation
        void simulationUpdate();
        void tf_broadcaster();
};

// Initialize
Arm::Arm()
{
    initialize();
}

void Arm::initialize()
{
    #ifndef SIMULATION
    std::string _port_name("/dev/ttyUSB0");
    int _baudrate = 1000000;
    _dxl_base.initialize(_port_name, _baudrate);
    #endif
    joint_offset.initialize(  0.0, 0.0, 159.0, 'o');
          joint0.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,   0.0, 1, _dxl_base, dynamixel_wrapper::PH54_200_S500_R, _operating_mode);
          joint1.initialize( 30.0, 0.0, 264.0, 'y', -atan(30.0/264.0)                  , -36.3, 2, _dxl_base, dynamixel_wrapper::H54_200_S500_R,  _operating_mode);
          joint2.initialize(-30.0, 0.0, 258.0, 'y',  atan(30.0/264.0)-atan(-30.0/258.0), -45.0, 3, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  _operating_mode);
          joint3.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,   0.0, 4, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  _operating_mode);
          joint4.initialize(  0.0, 0.0, 123.0, 'y',                   atan(-30.0/258.0),   0.0, 5, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  _operating_mode);
          joint5.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,   0.0, 6, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  _operating_mode);
    _pose_offset << joint0.getLink('x'), joint0.getLink('y'), joint0.getLink('z'), 0.0, 0.0, 0.0;
}

// Debag
void Arm::print()
{
    std::cout
    << "pose"
    << std::endl
    << _pose
    << std::endl
    << std::endl
    << "jacobian determinant"
    << std::endl
    << _jacobian.determinant()
    << std::endl
    << std::endl
    << "alternating euler determinant"
    << std::endl
    << _alternating_euler.determinant()
    << std::endl
    << std::endl
    << "angular velocity"
    << std::endl
    << inverseKinematics()
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

// Forward Kinematics
Eigen::Matrix<double, 3, 1> Arm::getPosition()
{
    _position = joint_offset.getRotationMatrix()*(joint_offset.getLink()+joint0.getRotationMatrix()*(joint0.getLink()+joint1.getRotationMatrix()*(joint1.getLink()+joint2.getRotationMatrix()*(joint2.getLink()+joint3.getRotationMatrix()*(joint3.getLink()+joint4.getRotationMatrix()*(joint4.getLink()+joint5.getRotationMatrix()*(joint5.getLink())))))));
    return _position;
}

Eigen::Matrix<double, 3, 1> Arm::getEulerAngle()
{
    _rotation_all = joint0.getRotationMatrix()*joint1.getRotationMatrix()*joint2.getRotationMatrix()*joint3.getRotationMatrix()*joint4.getRotationMatrix()*joint5.getRotationMatrix();

    // XYZ Euler
    // _euler(1,0) = asin(_rotation_all(0,2));
    // _euler(0,0) = acos(_rotation_all(2,2)/cos(_euler(1,0)));
    // if(-_rotation_all(1,2)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    // _euler(2,0) = acos(_rotation_all(0,0)/cos(_euler(1,0)));
    // if(-_rotation_all(0,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);

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
    getAngle();
    getPosition();
    getEulerAngle();
    _pose(0,0) = _position(0,0);
    _pose(1,0) = _position(1,0);
    _pose(2,0) = _position(2,0);
    _pose(3,0) = _euler(0,0);
    _pose(4,0) = _euler(1,0);
    _pose(5,0) = _euler(2,0);
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
    _target_pose(3,0) = target_pose.orientation.x;
    _target_pose(4,0) = target_pose.orientation.y;
    _target_pose(5,0) = target_pose.orientation.z;
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

double Arm::getDistance()
{
    return sqrt(pow((_target_pose(0,0)-_target_pose_start(0,0)),2)+pow((_target_pose(1,0)-_target_pose_start(1,0)),2)+pow((_target_pose(2,0)-_target_pose_start(2,0)),2));
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
    _translation_jacobian(0,0) = -d*s1*c2 -l2*s1*s2 +d*s1*c23 -l3*s1*s23 -l5*s1*c23*c4*s5 -l5*c1*s4*s5 -l5*s1*s23*c5;
    _translation_jacobian(0,1) = -d*c1*s2 +l2*c1*c2 +d*c1*s23 +l3*c1*c23 -l5*c1*s23*c4*s5 +l5*c1*c23*c5;
    _translation_jacobian(0,2) = d*c1*s23 +l3*c1*c23 -l5*c1*s23*c4*s5 +l5*c1*c23*c5;
    _translation_jacobian(0,3) = -l5*c1*c23*s4*s5 -l5*s1*c4*s5;
    _translation_jacobian(0,4) = l5*c1*c23*c4*s5 -l5*s1*s4*c5 -l5*c1*s23*s5;
    _translation_jacobian(0,5) = 0.0;

    _translation_jacobian(1,0) = d*c1*c2 +l2*c1*s2 -d*c1*c23 +l3*c1*s23 +l5*c1*c23*c4*s5 -l5*s1*s4*s5 +l5*c1*s23*c5;
    _translation_jacobian(1,1) = -d*s1*s2 +l2*s1*c2 +d*s1*s23 +l3*s1*c23 -l5*s1*s23*c4*s5 +l5*s1*c23*c5;
    _translation_jacobian(1,2) = d*s1*s23 +l3*s1*c23 -l5*s1*s23*c4*s5 +l5*s1*c23*c5;
    _translation_jacobian(1,3) = -l5*s1*c23*s4*s5 +l5*c1*c4*s5;
    _translation_jacobian(1,4) = l5*s1*c23*c4*c5 +l5*c1*s4*c5 -l5*s1*s23*s5;
    _translation_jacobian(1,5) = 0.0;

    _translation_jacobian(2,0) = 0.0;
    _translation_jacobian(2,1) = -d*c2 -l2*s2 +d*c23 -l3*s23 -l5*c23*c4*s5 -l5*s23*c5;
    _translation_jacobian(2,2) = d*c23 -l3*s23 -l5*c23*c4*s5 -l5*s23*c5;
    _translation_jacobian(2,3) = l5*s23*s4*s5;
    _translation_jacobian(2,4) = -l5*s23*c4*c5 -l5*c23*s5;
    _translation_jacobian(2,5) = 0.0;

    return _translation_jacobian;
}

Eigen::Matrix<double, 3, 6> Arm::getRotationJacobian()
{
    _alternating_euler <<
    0.0, -sin(_euler(0,0)), cos(_euler(0,0))*cos(_euler(1,0)),
    0.0,  cos(_euler(0,0)), sin(_euler(0,0))*cos(_euler(1,0)),
    1.0,               0.0,                 -sin(_euler(1,0));

    _alternating_rotation <<
    1.0, 0.0, 0.0,    c23,            s23*s4,                  c23*c5 -s23*c4*s5,
    0.0,  c1,  c1, s1*s23,  c1*c4 -s1*c23*s4,  c1*s4*s5 +s1*s23*c5 +s1*c23*c4*s5,
    0.0, -s1, -s1, c1*s23, -s1*c4 -c1*c23*s4, -s1*s4*s5 +c1*s23*c5 +c1*c23*c4*s5;

    _rotation_jacobian = _alternating_euler.inverse()*_alternating_rotation;
    return _rotation_jacobian;
}

void Arm::replaceVariables()
{
    d  = sqrt(pow(joint_offset.getLink('x'),2)+pow(joint_offset.getLink('y'),2)+pow(joint_offset.getLink('z'),2));
    l1 = sqrt(pow(joint0.getLink('x'),2)+pow(joint0.getLink('y'),2)+pow(joint0.getLink('z'),2));
    l2 = sqrt(pow(joint1.getLink('x'),2)+pow(joint1.getLink('y'),2)+pow(joint1.getLink('z'),2));
    l3 = sqrt(pow(joint2.getLink('x'),2)+pow(joint2.getLink('y'),2)+pow(joint2.getLink('z'),2));
    l4 = sqrt(pow(joint3.getLink('x'),2)+pow(joint3.getLink('y'),2)+pow(joint3.getLink('z'),2));
    l5 = sqrt(pow(joint4.getLink('x'),2)+pow(joint4.getLink('y'),2)+pow(joint4.getLink('z'),2));
    l6 = sqrt(pow(joint5.getLink('x'),2)+pow(joint5.getLink('y'),2)+pow(joint5.getLink('z'),2));

    c1 = cos(joint0.getGlobalAngle());
    c2 = cos(joint1.getGlobalAngle());
    c3 = cos(joint2.getGlobalAngle());
    c4 = cos(joint3.getGlobalAngle());
    c5 = cos(joint4.getGlobalAngle());
    c6 = cos(joint5.getGlobalAngle());
    c23 = cos(joint2.getGlobalAngle()+joint3.getGlobalAngle());

    s1 = sin(joint0.getGlobalAngle());
    s2 = sin(joint1.getGlobalAngle());
    s3 = sin(joint2.getGlobalAngle());
    s4 = sin(joint3.getGlobalAngle());
    s5 = sin(joint4.getGlobalAngle());
    s6 = sin(joint5.getGlobalAngle());
    s23 = sin(joint2.getGlobalAngle()+joint3.getGlobalAngle());
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
    
    // XYZ Euler
    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "arm_base_link";
    // transformStamped.child_frame_id = "eulerX";
    // transformStamped.transform.translation.x = _position(0,0)/1000.0;
    // transformStamped.transform.translation.y = _position(1,0)/1000.0;
    // transformStamped.transform.translation.z = _position(2,0)/1000.0;
    
    // q.setRPY(_euler(0,0), 0, 0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "eulerX";
    // transformStamped.child_frame_id = "eulerXY";
    // transformStamped.transform.translation.x = 0;
    // transformStamped.transform.translation.y = 0;
    // transformStamped.transform.translation.z = 0;
    
    // q.setRPY(0, _euler(1,0), 0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "eulerXY";
    // transformStamped.child_frame_id = "eulerXYZ";
    // transformStamped.transform.translation.x = 0;
    // transformStamped.transform.translation.y = 0;
    // transformStamped.transform.translation.z = 0;
    
    // q.setRPY(0, 0, _euler(2,0));
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);
    
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