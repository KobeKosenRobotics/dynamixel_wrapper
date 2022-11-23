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

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    operating_mode = msg->orientation.w;
    if(operating_mode < 0.5) joint_angle << msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    arm.setTargetPose(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Subscriber
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);

    while(nh.ok())
    {
        arm.simulationUpdate();
        arm.getPose();

        if(operating_mode < 0.5) arm.setAngle(joint_angle);
        arm.setAngularVelocity(arm.inverseKinematics());
        
        arm.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}