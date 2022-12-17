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
std_msgs::Float32MultiArray motor_angular_velocity;

// Subscriber
std_msgs::Float32MultiArray sensor_angle;
geometry_msgs::Pose target_pose;
std_msgs::Bool simulator_enable;
std_msgs::Bool exc_enable;
std_msgs::Bool emergency_stop;

void sensor_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    sensor_angle = *msg;
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    target_pose = *msg;
    motor_angular_velocity.data[0] = target_pose.position.x;
    motor_angular_velocity.data[1] = target_pose.position.y;
    motor_angular_velocity.data[2] = target_pose.position.z;
    motor_angular_velocity.data[3] = target_pose.orientation.x;
    motor_angular_velocity.data[4] = target_pose.orientation.y;
    motor_angular_velocity.data[5] = target_pose.orientation.z;
}

void simulator_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    simulator_enable = *msg;
}

void exc_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    exc_enable = *msg;
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
    ros::Publisher motor_angular_velocity_pub = nh.advertise<std_msgs::Float32MultiArray>("motor_angular_velocity", 100);
    motor_angular_velocity.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber sensor_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("sensor_angle", 100, sensor_angle_cb);
    sensor_angle.data.resize(JOINT_NUMBER);
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);
    ros::Subscriber simulator_enable_sub = nh.subscribe<std_msgs::Bool>("simulator_enable", 10, simulator_enable_cb);
    ros::Subscriber exc_enable_sub = nh.subscribe<std_msgs::Bool>("exc_enable", 10, exc_enable_cb);
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);

    while(nh.ok())
    {
        // calculate()

        motor_angular_velocity_pub.publish(motor_angular_velocity);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}