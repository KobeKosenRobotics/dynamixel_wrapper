#include "class_exc_arm_property.hpp"
#include "class_exc_arm.hpp"

#include "class_wait.hpp"

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

// Global
ExCArm exc_arm;
Wait wait;
int step = 5;

// Publisher
std_msgs::Float32MultiArray CSAV;
std_msgs::Float32MultiArray CMAV;

// Subscriber
std_msgs::Float32MultiArray SCA;
std_msgs::Float32MultiArray MCA;
std_msgs::Float32MultiArray target_angle;
geometry_msgs::Pose target_pose;
std_msgs::Bool motor_enable;
std_msgs::Bool exc_enable;
std_msgs::Bool emergency_stop;

// Function
void sequence()
{
    switch(step)
    {
    case 0:
        /* code */
        break;

    case 5:
        exc_arm.setCalculationMode(0);
        exc_arm.setTargetAngle(0.0, 0.2, 0.7, 0.0, 0.2, 0.0);
        step = 10;
        break;

    case 10:
        if(exc_arm.isInTargetAngle())
        {
            step = 15;
        }
        break;

    case 15:
        exc_arm.setCalculationMode(2);
        exc_arm.setTargetPose(500.0, 0.0, 500.0, 0.0, 1.5, 0.0);
        step = 20;
        exc_arm.measurementStart();

    case 20:
        if(!wait.isWaiting(exc_arm.getDurationTIme()))
        {
            exc_arm.measurementEnd();
            step = 5;
        }

    default:
        break;
    }
};

void SCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(!exc_arm.getMotorEnable())
    {
        exc_arm.setSensorAngle(*msg);
    }
}

void MCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(exc_arm.getMotorEnable())
    {
        exc_arm.setSensorAngle(*msg);
    }
}

void target_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    // exc_arm.setTargetAngle(*msg);
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    // exc_arm.setTargetPose(*msg);
}

void motor_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    exc_arm.setMotorEnable(*msg);
}

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    exc_arm.setEmergencyStop(*msg);
}

void calculation_mode_cb(std_msgs::Int16::ConstPtr msg)
{
    // exc_arm.setCalculationMode(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExC_Calculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher CSAV_pub = nh.advertise<std_msgs::Float32MultiArray>("CSAV", 100);
    CSAV.data.resize(JOINT_NUMBER);
    ros::Publisher CMAV_pub = nh.advertise<std_msgs::Float32MultiArray>("CMAV", 100);
    CMAV.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber SCA_sub = nh.subscribe<std_msgs::Float32MultiArray>("SCA", 100, SCA_cb);
    SCA.data.resize(JOINT_NUMBER);
    ros::Subscriber MCA_sub = nh.subscribe<std_msgs::Float32MultiArray>("MCA", 100, MCA_cb);
    MCA.data.resize(JOINT_NUMBER);
    ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_angle", 10, target_angle_cb);
    target_angle.data.resize(JOINT_NUMBER);
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("motor_enable", 10, motor_enable_cb);
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);
    ros::Subscriber calculation_mode_sub = nh.subscribe<std_msgs::Int16>("calculation_mode", 10, calculation_mode_cb);

    while(nh.ok())
    {
        // exc_arm.print();

        sequence();

        if(exc_arm.getMotorEnable())
        {
            CSAV_pub.publish(exc_arm.getMotorAngularVelocityZero());
            CMAV_pub.publish(exc_arm.getMotorAngularVelocity());
        }
        else
        {
            CSAV_pub.publish(exc_arm.getMotorAngularVelocity());
            CMAV_pub.publish(exc_arm.getMotorAngularVelocityZero());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}