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

    public:
        Arm();

        void initialize();
        
        Joint joint_offset, joint0, joint1, joint2, joint3, joint4, joint5;
        
        void setAngle(Eigen::Matrix<double, 6, 1> angle_rad);
        void print();
        void update();
        void forwardKinematics();
        void inverseKinematics();
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
          joint0.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0, 1, _dxl_base, dynamixel_wrapper::PH54_200_S500_R, 4);
          joint1.initialize( 30.0, 0.0, 264.0, 'y',  atan(30.0/264.0)                  , 2, _dxl_base, dynamixel_wrapper::H54_200_S500_R,  4);
          joint2.initialize(-30.0, 0.0, 258.0, 'y', -atan(30.0/264.0)+atan(-30.0/258.0), 3, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  4);
          joint3.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0, 4, _dxl_base, dynamixel_wrapper::H54_100_S500_R,  4);
          joint4.initialize(  0.0, 0.0, 123.0, 'y',                  -atan(-30.0/258.0), 5, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  4);
          joint5.initialize(  0.0, 0.0,   0.0, 'z',                                 0.0, 6, _dxl_base, dynamixel_wrapper::H42_020_S300_R,  4);
    
}

void Arm::setAngle(Eigen::Matrix<double, 6, 1> angle_rad)
{
    joint0.setGoalPosition(angle_rad(0,0));
    joint1.setGoalPosition(angle_rad(1,0));
    joint2.setGoalPosition(angle_rad(2,0));
    joint3.setGoalPosition(angle_rad(3,0));
    joint4.setGoalPosition(angle_rad(4,0));
    joint5.setGoalPosition(angle_rad(5,0));
}

void Arm::print()
{
    std::cout << 
    joint0.simulationAngle('z') << "  " << 
    joint1.simulationAngle('y') << "  " << 
    joint2.simulationAngle('y') << "  " << 
    joint3.simulationAngle('z') << "  " << 
    joint4.simulationAngle('y') << "  " << 
    joint5.simulationAngle('z') << std::endl;
}

void Arm::tf_broadcaster()
{
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;
    
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