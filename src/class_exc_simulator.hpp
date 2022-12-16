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
    transformStamped.child_frame_id = exc_arm_property.getJointName(0);
    transformStamped.transform.translation.x = exc_arm_property.getLink(0,0)/1000.0;
    transformStamped.transform.translation.y = exc_arm_property.getLink(0,1)/1000.0;
    transformStamped.transform.translation.z = exc_arm_property.getLink(0,2)/1000.0;

    q.setRPY(_joint[0].getSimulationAngle(0), _joint[0].getSimulationAngle(1), _joint[0].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 1 ~
    for(int i = 1; i < JOINT_NUMBER+1; i++)
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = exc_arm_property.getJointName(i-1);
        transformStamped.child_frame_id = exc_arm_property.getJointName(i);
        transformStamped.transform.translation.x = exc_arm_property.getLink(i,0)/1000.0;
        transformStamped.transform.translation.y = exc_arm_property.getLink(i,1)/1000.0;
        transformStamped.transform.translation.z = exc_arm_property.getLink(i,2)/1000.0;

        q.setRPY(_joint[i].getSimulationAngle(0), _joint[i].getSimulationAngle(1), _joint[i].getSimulationAngle(2));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }
}

#endif