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

Eigen::Matrix<double, 6, 1> rotation;

void joint_rotation_cb(geometry_msgs::Pose::ConstPtr msg)
{
    rotation << msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Subscriber
    ros::Subscriber joint_rotation_sub = nh.subscribe<geometry_msgs::Pose>("joint_rotation", 10, joint_rotation_cb);

    Arm arm;

    while(nh.ok())
    {
        arm.setAngularVelocity(rotation);

        arm.simulationUpdate();
        arm.tf_broadcaster();
        
        arm.getPose();
        
        arm.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}