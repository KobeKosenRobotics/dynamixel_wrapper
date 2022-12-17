#include "class_exc_arm_property.hpp"

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

// Publisher
std_msgs::Float32MultiArray angular_velocity;

// Subscriber
std_msgs::Float32MultiArray angle;
geometry_msgs::Pose target_pose;
std_msgs::Int16 operating_mode;
std_msgs::Bool emergency_stop;

void angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    angle = *msg;
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    target_pose = *msg;
    angular_velocity.data[0] = target_pose.position.x;
    angular_velocity.data[1] = target_pose.position.y;
    angular_velocity.data[2] = target_pose.position.z;
    angular_velocity.data[3] = target_pose.orientation.x;
    angular_velocity.data[4] = target_pose.orientation.y;
    angular_velocity.data[5] = target_pose.orientation.z;
}

void operating_mode_cb(std_msgs::Int16::ConstPtr msg)
{
    operating_mode = *msg;
}

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    emergency_stop = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExC_Calculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher angular_velocity_pub = nh.advertise<std_msgs::Float32MultiArray>("angular_velocity", 100);
    angular_velocity.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("angle", 100, angle_cb);
    angle.data.resize(JOINT_NUMBER);
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);
    ros::Subscriber operating_mode_sub = nh.subscribe<std_msgs::Int16>("operating_mode", 10, operating_mode_cb);
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);

    while(nh.ok())
    {
        // calculate()

        angular_velocity_pub.publish(angular_velocity);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}