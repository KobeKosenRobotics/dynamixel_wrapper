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
const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class ExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+1, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+1> _joint_position;
        Eigen::Matrix<int, 3, JOINT_NUMBER+1> _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+1, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;

    public:
        ExCArmProperty();

        Eigen::Matrix<double, 3, JOINT_NUMBER+1> getJointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link);
        Eigen::Matrix<double, 3, 1> getLink(int joint);
        double getLink(int joint, int axis);
        int getRotationAxis(int joint);
        Eigen::Matrix<double, 3, 3> getRotationMatrix(int joint, double angle);
        std::string getJointName(int joint);
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> getProportionalGainAngleOperating();
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

    _proportional_gain_angle_operating <<
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.0;
}

Eigen::Matrix<double, 3, JOINT_NUMBER+1> ExCArmProperty::getJointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link)
{
    for(int column = 0; column < 3; column++)
    {
        for(int row = 1; row < (JOINT_NUMBER+1); row++)
        {
            link(row, column) += link(row-1,column);
        }
    }

    return link.transpose();
}

Eigen::Matrix<double, 3, 1> ExCArmProperty::getLink(int joint)
{
    Eigen::Matrix<double, 3, 1> link_;
    link_ << getLink(joint, 0), getLink(joint, 1), getLink(joint, 2);
    return link_;
}

double ExCArmProperty::getLink(int joint, int axis)
{
    return _link(joint, axis);
}

int ExCArmProperty::getRotationAxis(int joint)
{
    if(_rotation_axis(0,joint) == 1) return 0;
    else if(_rotation_axis(1,joint) == 1) return 1;
    else if(_rotation_axis(2,joint) == 1) return 2;
    else return -1;
}

Eigen::Matrix<double, 3, 3> ExCArmProperty::getRotationMatrix(int joint, double angle)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_;

    if(getRotationAxis(joint) == 0)
    {
        rotation_matrix_ <<
        1.0,        0.0,         0.0,
        0.0, cos(angle), -sin(angle),
        0.0, sin(angle),  cos(angle);
    }
    else if(getRotationAxis(joint) == 1)
    {
        rotation_matrix_ <<
         cos(angle),        0.0, sin(angle),
                0.0,        1.0,        0.0,
        -sin(angle),        0.0, cos(angle);
    }
    else if(getRotationAxis(joint) == 2)
    {
        rotation_matrix_ <<
        cos(angle), -sin(angle), 0.0,
        sin(angle),  cos(angle), 0.0,
               0.0,         0.0, 1.0;
    }
    else
    {
        rotation_matrix_ <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    }
    return rotation_matrix_;
}

std::string ExCArmProperty::getJointName(int joint)
{
    return _joint_name(joint, 0);
}

Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> ExCArmProperty::getProportionalGainAngleOperating()
{
    return _proportional_gain_angle_operating;
}

#endif