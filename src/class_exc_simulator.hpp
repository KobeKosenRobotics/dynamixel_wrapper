#ifndef CLASS_EXC_SIMULATOR_HPP
#define CLASS_EXC_SIMULATOR_HPP

#include "class_exc_arm_property.hpp"
#include "class_exc_joint_simulator.hpp"

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

class ExCSimulator
{
    private:
        ExCJointSimulator _joint[JOINT_NUMBER+1];
        std_msgs::Float32MultiArray _angle, _angular_velocity;

        // Function
        void tfBroadcaster();

    public:
        ExCSimulator();
        void update(std_msgs::Float32MultiArray angular_velocity);
        std_msgs::Float32MultiArray getAngle();
};

// Public
ExCSimulator::ExCSimulator()
{
    _angle.data.resize(JOINT_NUMBER);
    _angular_velocity.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER+1; i++)
    {
        _joint[i].initialize(i);
    }
}

void ExCSimulator::update(std_msgs::Float32MultiArray angular_velocity)
{
    _angular_velocity = angular_velocity;

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _joint[i].angularVelocity2Angle(_angular_velocity.data[i]);
    }

    tfBroadcaster();
}

std_msgs::Float32MultiArray ExCSimulator::getAngle()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _angle.data[i] = _joint[i].getSimulationAngle();
    }
    return _angle;
}

// Private
void ExCSimulator::tfBroadcaster()
{
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Joint 0
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "joint0";
    transformStamped.transform.translation.x = exc_arm_property.getLink(0,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(0,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(0,2)/1000.0;

    q.setRPY(_joint[0].getSimulationAngle(0), _joint[0].getSimulationAngle(1), _joint[0].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 1
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint0";
    transformStamped.child_frame_id = "joint1";
    transformStamped.transform.translation.x = exc_arm_property.getLink(1,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(1,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(1,2)/1000.0;

    q.setRPY(_joint[1].getSimulationAngle(0), _joint[1].getSimulationAngle(1), _joint[1].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 2
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint1";
    transformStamped.child_frame_id = "joint2";
    transformStamped.transform.translation.x = exc_arm_property.getLink(2,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(2,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(2,2)/1000.0;

    q.setRPY(_joint[2].getSimulationAngle(0), _joint[2].getSimulationAngle(1), _joint[2].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 3
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint2";
    transformStamped.child_frame_id = "joint3";
    transformStamped.transform.translation.x = exc_arm_property.getLink(3,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(3,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(3,2)/1000.0;

    q.setRPY(_joint[3].getSimulationAngle(0), _joint[3].getSimulationAngle(1), _joint[3].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 4
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint3";
    transformStamped.child_frame_id = "joint4";
    transformStamped.transform.translation.x = exc_arm_property.getLink(4,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(4,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(4,2)/1000.0;

    q.setRPY(_joint[4].getSimulationAngle(0), _joint[4].getSimulationAngle(1), _joint[4].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 5
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint4";
    transformStamped.child_frame_id = "joint5";
    transformStamped.transform.translation.x = exc_arm_property.getLink(5,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(5,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(5,2)/1000.0;

    q.setRPY(_joint[5].getSimulationAngle(0), _joint[5].getSimulationAngle(1), _joint[5].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 6
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint5";
    transformStamped.child_frame_id = "joint6";
    transformStamped.transform.translation.x = exc_arm_property.getLink(6,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(6,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(6,2)/1000.0;

    q.setRPY(_joint[6].getSimulationAngle(0), _joint[6].getSimulationAngle(1), _joint[6].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

#endif