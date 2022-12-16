#ifndef CLASS_EXC_ARM_HPP
#define CLASS_EXC_ARM_HPP

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

class ExCArm
{
    public:

    private:
};


Eigen::Matrix<double, 3, JOINT_NUMBER+1> ExCArm::getJointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link)
{
    for(int colum = 0; colum < 3; colum++)
    {
        for(int row = 1; row < (JOINT_NUMBER+1); row++)
        {
            link(row, colum) += link(row-1,colum);
        }
    }

    std::cout << link << std::endl;

    return link.transpose();
}

// Simulation
double ExCArm::getLink(int joint, int axis)
{
    return _link(joint, axis);
}

#endif