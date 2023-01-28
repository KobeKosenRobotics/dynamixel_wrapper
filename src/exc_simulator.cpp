#include "class_exc_arm_property.hpp"
#include "class_exc_joint_simulator.hpp"
#include "class_exc_arm_simulator.hpp"

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
ExCArmSimulator exc_simulator;

// Publisher
std_msgs::Float32MultiArray SCA;

// Subscriber
std_msgs::Float32MultiArray CSAV;

// Function
void CSAV_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    CSAV = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExCArmSimulator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher SCA_pub = nh.advertise<std_msgs::Float32MultiArray>("SCA", 100);
    SCA.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber CSAV_sub = nh.subscribe<std_msgs::Float32MultiArray>("CSAV", 100, CSAV_cb);
    CSAV.data.resize(JOINT_NUMBER);

    while(nh.ok())
    {
        exc_simulator.update(CSAV);

        SCA_pub.publish(exc_simulator.getAngle());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}