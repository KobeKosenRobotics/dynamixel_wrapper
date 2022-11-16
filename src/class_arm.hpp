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

        dynamixel_wrapper::dynamixel_wrapper_base _dxl_base;

        Eigen::Matrix<double, 6, 1> _sensor_angle, _sensor_angular_velocity;
        Eigen::Matrix<double, 6, 1> _motor_angle, _motor_angular_velocity;

        Eigen::Matrix<double, 3, 3> _rotation_all;
        Eigen::Matrix<double, 3, 1> _position, _euler;
        Eigen::Matrix<double, 6, 6> _jacobi;
        Eigen::Matrix<double, 6, 1> _pose, _target_pose;

    public:
        Arm();
        void initialize();
        Joint joint_offset, joint0, joint1, joint2, joint3, joint4, joint5;
        
        void print();

        Eigen::Matrix<double, 6, 1> getAngle();
        Eigen::Matrix<double, 6, 1> getAngularVelocity();
        void setAngle(Eigen::Matrix<double, 6, 1> angle_rad);
        void setAngularVelocity(Eigen::Matrix<double, 6, 1> angular_velocity_radps);

        void getPosition();
        void getEulerAngle();
        void getPose();

        Eigen::Matrix<double, 6, 1> inverseKinematics();
        Eigen::Matrix<double, 6, 6> getJacobian();
        Eigen::Matrix<double, 3, 6> getTranslationJacobian();
        Eigen::Matrix<double, 3, 6> getRotationJacobian();

        // Simulation
        void tf_broadcaster();
};

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
          joint0.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,  0.0, 1, _dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
          joint1.initialize( 30.0, 0.0, 264.0, 'y',  atan(30.0/264.0)                  , 36.3, 2, _dxl_base, dynamixel_wrapper::H54_200_S500_R,  1);
          joint2.initialize(-30.0, 0.0, 258.0, 'y', -atan(30.0/264.0)+atan(-30.0/258.0), 45.0, 3, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  1);
          joint3.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,  0.0, 4, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  1);
          joint4.initialize(  0.0, 0.0, 123.0, 'y',                  -atan(-30.0/258.0),  0.0, 5, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  1);
          joint5.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0,  0.0, 6, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  1);
}

void Arm::print()
{
    std::cout
    << _pose
    << std::endl;
}

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

void Arm::getPosition()
{
    _position = joint_offset.getRotationMatrix()*(joint_offset.getLink()+joint0.getRotationMatrix()*(joint0.getLink()+joint1.getRotationMatrix()*(joint1.getLink()+joint2.getRotationMatrix()*(joint2.getLink()+joint3.getRotationMatrix()*(joint3.getLink()+joint4.getRotationMatrix()*(joint4.getLink()+joint5.getRotationMatrix()*(joint5.getLink())))))));
}

void Arm::getEulerAngle()
{
    _rotation_all = joint0.getRotationMatrix()*joint1.getRotationMatrix()*joint2.getRotationMatrix()*joint3.getRotationMatrix()*joint4.getRotationMatrix()*joint5.getRotationMatrix();
    _euler(1,0) = asin(_rotation_all(0,2));
    _euler(0,0) = acos(_rotation_all(2,2)/cos(_euler(1,0)));
    if(-_rotation_all(1,2)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    _euler(2,0) = acos(_rotation_all(0,0)/cos(_euler(1,0)));
    if(-_rotation_all(0,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);

    // Eigen::Matrix<double, 3, 3> rx, ry, rz, re;
    // rx << 1.0,              0.0,               0.0, 
    //       0.0, cos(_euler(0,0)), -sin(_euler(0,0)),
    //       0.0, sin(_euler(0,0)),  cos(_euler(0,0));
    
    // ry <<  cos(_euler(1,0)), 0.0, sin(_euler(1,0)),
    //                     0.0, 1.0,              0.0,
    //       -sin(_euler(1,0)), 0.0, cos(_euler(1,0));
    
    // rz << cos(_euler(2,0)), -sin(_euler(2,0)), 0.0,
    //       sin(_euler(2,0)),  cos(_euler(2,0)), 0.0,
    //                    0.0,               0.0, 1.0;
    
    // re = rx*ry*rz;
    // std::cout << _rotation_all-re << std::endl << std::endl;
}

void Arm::getPose()
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
}

// Eigen::Matrix<double, 6, 1> Arm::inverseKinematics()
// {
//     return
// }

void Arm::tf_broadcaster()
{
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Euler
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "euler";
    transformStamped.transform.translation.x = _position(0,0)/1000.0;
    transformStamped.transform.translation.y = _position(1,0)/1000.0;
    transformStamped.transform.translation.z = _position(2,0)/1000.0;
    
    q.setRPY(_euler(2,0), _euler(1,0), _euler(1,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "euler1";
    transformStamped.transform.translation.x = _position(0,0)/1000.0;
    transformStamped.transform.translation.y = _position(1,0)/1000.0;
    transformStamped.transform.translation.z = _position(2,0)/1000.0;
    
    q.setRPY(_euler(0,0), 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "euler1";
    transformStamped.child_frame_id = "euler2";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    
    q.setRPY(0, _euler(1,0), 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "euler2";
    transformStamped.child_frame_id = "euler3";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    
    q.setRPY(0, 0, _euler(2,0));
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