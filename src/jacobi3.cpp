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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm");
    ros::NodeHandle nh;
    double rate = 10000.0;
    ros::Rate loop_rate(rate);

    Arm arm;
    Eigen::Matrix<double, 6, 1> angle;
    // angle << 0.0, M_PI/3.0, M_PI/3.0, 0.0, M_PI/3.0, 0.0;
    
    // angle << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    while(nh.ok())
    {
        angle << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        arm.setAngle(angle);
        arm.tf_broadcaster();
        arm.print();
        angle << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}