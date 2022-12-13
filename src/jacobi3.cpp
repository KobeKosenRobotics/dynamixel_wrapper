#define SIMULATION

#include "class_joint.hpp"
#include "class_arm.hpp"

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

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI;

Arm arm;
Eigen::Matrix<double, 6, 1> joint_angle;
int operating_mode;

void angle_cb(geometry_msgs::Pose::ConstPtr msg)
{
    #ifndef SIMULATION
    arm.getAngle(*msg);
    #endif
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    arm.setTarget(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher angular_pub = nh.advertise<geometry_msgs::Pose>("angular_velocity", 100);
    geometry_msgs::Pose angular_velocity;

    // Subscriber
    ros::Subscriber angle_sub = nh.subscribe<geometry_msgs::Pose>("angle", 100, angle_cb);
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);

    while(nh.ok())
    {
        #ifdef SIMULATION
        arm.getAngle();
        #endif
        arm.update();

        angular_velocity.position.x = arm.joint0.getMotorAngularVelocity();
        angular_velocity.position.y = arm.joint1.getMotorAngularVelocity();
        angular_velocity.position.z = arm.joint2.getMotorAngularVelocity();
        angular_velocity.orientation.x = arm.joint3.getMotorAngularVelocity();
        angular_velocity.orientation.y = arm.joint4.getMotorAngularVelocity();
        angular_velocity.orientation.z = arm.joint5.getMotorAngularVelocity();

        angular_pub.publish(angular_velocity);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}