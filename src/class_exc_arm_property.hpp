#ifndef CLASS_EXC_ARM_PROPERTY
#define CLASS_EXC_ARM_PROPERTY

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

const int JOINT_NUMBER = 6;

class ExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+1, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+1> _joint_position;
        Eigen::Matrix<int, 3, JOINT_NUMBER+1> _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+1, 1> _joint_name;

    public:
        ExCArmProperty();

        Eigen::Matrix<double, 3, JOINT_NUMBER+1> getJointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link);
        double getLink(int joint, int axis);
        int getAxis(int joint);
        std::string getJointName(int joint);
};
ExCArmProperty exc_arm_property;

ExCArmProperty::ExCArmProperty()
{
    _link <<
      0.0, 0.0, 159.0,
      0.0, 0.0,   0.0,
     30.0, 0.0, 264.0,
    -30.0, 0.0, 258.0,
      0.0, 0.0,   0.0,
      0.0, 0.0, 123.0,
      0.0, 0.0,   0.0;

    _joint_position = getJointPosition(_link);

    _translation_axis <<
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;

    _rotation_axis <<
    0, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 0, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 0;

    _joint_name << "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6";
}

Eigen::Matrix<double, 3, JOINT_NUMBER+1> ExCArmProperty::getJointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link)
{
    for(int colum = 0; colum < 3; colum++)
    {
        for(int row = 1; row < (JOINT_NUMBER+1); row++)
        {
            link(row, colum) += link(row-1,colum);
        }
    }

    return link.transpose();
}

double ExCArmProperty::getLink(int joint, int axis)
{
    return _link(joint, axis);
}

int ExCArmProperty::getAxis(int joint)
{
    if(_rotation_axis(0,joint) == 1) return 0;
    else if(_rotation_axis(1,joint) == 1) return 1;
    else if(_rotation_axis(2,joint) == 1) return 2;
    else return -1;
}

std::string ExCArmProperty::getJointName(int joint)
{
    return _joint_name(joint, 0);
}

#endif