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
#include <Eigen/SVD>

// #define DOF6
#define DOFN

// #define COLLECT_RANDOM_TARGET

#ifdef DOF6
const int JOINT_NUMBER = 6;
#endif

#ifndef DOF6

#ifdef DOFN
const int JOINT_NUMBER = 30;
#endif

#endif

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class ExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+1, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+1> _joint_position;
        Eigen::Matrix<double, 3, JOINT_NUMBER+1> _translation_axis, _rotation_axis;
        Eigen::Matrix<double, 3, 3> _gst_zero_rotation_matrix;
        Eigen::Matrix<double, 4, 4> _gst_zero;
        Eigen::Matrix<std::string, JOINT_NUMBER+1, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;
        Eigen::Matrix<double, JOINT_NUMBER, 1> _initial_target_angle;
        Eigen::Matrix<double, JOINT_NUMBER, 2> _joint_angle_limit;

    public:
        ExCArmProperty();

        Eigen::Matrix<double, 3, JOINT_NUMBER+1> link2JointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link);
        Eigen::Matrix<double, 3, 1> getLink(int joint);
        double getLink(int joint, int axis);
        int getRotationAxis(int joint);
        Eigen::Matrix<double, 3, 3> getRotationMatrix(int joint, double angle);
            Eigen::Matrix<double, 3, 3> getRotationMatrixX(double angle);
            Eigen::Matrix<double, 3, 3> getRotationMatrixY(double angle);
            Eigen::Matrix<double, 3, 3> getRotationMatrixZ(double angle);
        Eigen::Matrix<double, JOINT_NUMBER, 1> getInitialTargetAngle();
        double getLowerAngleLimit(int joint_);
        double getUpperAngleLimit(int joint_);

        // ExC Joint
        Eigen::Matrix<double, 3, 1> getQ(int joint);
        Eigen::Matrix<double, 3, 1> getV(int joint);
        Eigen::Matrix<double, 3, 1> getW(int joint);
        Eigen::Matrix<double, 4, 4> getGstZero();

        // Simulator
        std::string getJointName(int joint);

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> getProportionalGainAngleOperating();

        // Pseudo-Inverse Matrix
        Eigen::Matrix<double, JOINT_NUMBER, 6> getPseudoInverseMatrix(Eigen::Matrix<double, 6, JOINT_NUMBER> matrix_);

        // Rank
        int getRank(Eigen::Matrix<double, 6, JOINT_NUMBER> matrix_);
};
ExCArmProperty exc_arm_property;

ExCArmProperty::ExCArmProperty()
{
    #ifdef DOF6
    _link <<
      0.0, 0.0, 159.0,
      0.0, 0.0,   0.0,
     30.0, 0.0, 264.0,
    -30.0, 0.0, 258.0,
      0.0, 0.0,   0.0,
      0.0, 0.0, 123.0,
      0.0, 0.0,   0.0;

    _joint_position = link2JointPosition(_link);

    _translation_axis <<
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;


    _rotation_axis <<
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0;

    _joint_name << "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6";

    _proportional_gain_angle_operating <<
    10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 10.0;

    _joint_angle_limit <<
    -    M_PI    , M_PI    ,
    -    M_PI/2.0, M_PI/2.0,
    -3.0*M_PI/4.0, M_PI/2.0,
    -    M_PI    , M_PI    ,
    -    M_PI/2.0, M_PI/2.0,
    -    M_PI    , M_PI    ;

    #endif

    #ifndef DOF6

    #ifdef DOFN
    for(int i = 0; i < JOINT_NUMBER+1; i++)
    {
        // _link(i,2) = double(1000.0/(JOINT_NUMBER+1));
        _link(i,2) = 300.0;

        if(i < JOINT_NUMBER)
        {
            if(i%2 == 0)
            {
                _rotation_axis(2,i) = 1.0;
            }
            else
            {
                _rotation_axis(1,i) = 1.0;
                _initial_target_angle(i,0) = 0.1;
            }

            _joint_angle_limit(i,0) = -M_PI;
            _joint_angle_limit(i,1) = M_PI;
        }

        std::stringstream ss;
        ss << "joint" << i;
        _joint_name(i,0) = ss.str();

    }

    _proportional_gain_angle_operating.setIdentity();

    _joint_position = link2JointPosition(_link);
    #endif

    #endif

    _gst_zero_rotation_matrix.setIdentity();

    _gst_zero <<
    _gst_zero_rotation_matrix(0,0), _gst_zero_rotation_matrix(0,1), _gst_zero_rotation_matrix(0,2), _joint_position(0, JOINT_NUMBER),
    _gst_zero_rotation_matrix(1,0), _gst_zero_rotation_matrix(1,1), _gst_zero_rotation_matrix(1,2), _joint_position(1, JOINT_NUMBER),
    _gst_zero_rotation_matrix(2,0), _gst_zero_rotation_matrix(2,1), _gst_zero_rotation_matrix(2,2), _joint_position(2, JOINT_NUMBER),
                               0.0,                            0.0,                            0.0,                              1.0;
}

Eigen::Matrix<double, 3, JOINT_NUMBER+1> ExCArmProperty::link2JointPosition(Eigen::Matrix<double, JOINT_NUMBER+1, 3> link)
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
    Eigen::Matrix<double, 3, 3> rotation_matrix_;

    if(joint >= JOINT_NUMBER)
    {
        rotation_matrix_ = _gst_zero_rotation_matrix;
    }
    else if(getRotationAxis(joint) == 0)
    {
        rotation_matrix_ = getRotationMatrixX(angle);
    }
    else if(getRotationAxis(joint) == 1)
    {
        rotation_matrix_ = getRotationMatrixY(angle);
    }
    else if(getRotationAxis(joint) == 2)
    {
        rotation_matrix_ = getRotationMatrixZ(angle);
    }
    else
    {
        rotation_matrix_.setIdentity();
    }
    return rotation_matrix_;
}

Eigen::Matrix<double, 3, 3> ExCArmProperty::getRotationMatrixX(double angle)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_x_;
    rotation_matrix_x_ <<
    1.0,        0.0,         0.0,
    0.0, cos(angle), -sin(angle),
    0.0, sin(angle),  cos(angle);
    return rotation_matrix_x_;
}

Eigen::Matrix<double, 3, 3> ExCArmProperty::getRotationMatrixY(double angle)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_y_;
    rotation_matrix_y_ <<
     cos(angle),        0.0, sin(angle),
            0.0,        1.0,        0.0,
    -sin(angle),        0.0, cos(angle);
    return rotation_matrix_y_;
}

Eigen::Matrix<double, 3, 3> ExCArmProperty::getRotationMatrixZ(double angle)
{
    Eigen::Matrix<double, 3,3> rotation_matrix_z_;
    rotation_matrix_z_ <<
    cos(angle), -sin(angle), 0.0,
    sin(angle),  cos(angle), 0.0,
           0.0,         0.0, 1.0;
    return rotation_matrix_z_;
}

Eigen::Matrix<double, JOINT_NUMBER, 1> ExCArmProperty::getInitialTargetAngle()
{
    return _initial_target_angle;
}

double ExCArmProperty::getLowerAngleLimit(int joint_)
{
    return _joint_angle_limit(joint_, 0);
}

double ExCArmProperty::getUpperAngleLimit(int joint_)
{
    return _joint_angle_limit(joint_,1);
}

// ExC Joint
Eigen::Matrix<double, 3, 1> ExCArmProperty::getQ(int joint)
{
    Eigen::Matrix<double, 3, 1> q_;
    q_ << _joint_position(0, joint), _joint_position(1, joint), _joint_position(2, joint);
    return q_;
}

Eigen::Matrix<double, 3, 1> ExCArmProperty::getV(int joint)
{
    Eigen::Matrix<double, 3, 1> v_;
    v_ << _translation_axis(0, joint), _translation_axis(1, joint), _translation_axis(2, joint);
    return v_;
}

Eigen::Matrix<double, 3, 1> ExCArmProperty::getW(int joint)
{
    Eigen::Matrix<double, 3, 1> w_;
    w_ << _rotation_axis(0, joint), _rotation_axis(1, joint), _rotation_axis(2, joint);
    return w_;
}

Eigen::Matrix<double, 4, 4> ExCArmProperty::getGstZero()
{
    return _gst_zero;
}

std::string ExCArmProperty::getJointName(int joint)
{
    return _joint_name(joint, 0);
}

Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> ExCArmProperty::getProportionalGainAngleOperating()
{
    return _proportional_gain_angle_operating;
}

// Pseudo-Inverse Matrix
Eigen::Matrix<double, JOINT_NUMBER, 6> ExCArmProperty::getPseudoInverseMatrix(Eigen::Matrix<double, 6, JOINT_NUMBER> matrix_)
{
    Eigen::Matrix<double, JOINT_NUMBER, 6> pseudo_inverse_matrix_;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd S = svd.singularValues().asDiagonal();
    Eigen::MatrixXd V = svd.matrixV();

    pseudo_inverse_matrix_ = V*S.inverse()*U.transpose();

    // std::cout << pseudo_inverse_matrix_*matrix_ << std::endl;

    return pseudo_inverse_matrix_;
}

int ExCArmProperty::getRank(Eigen::Matrix<double, 6, JOINT_NUMBER> matrix_)
{
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(matrix_);
    int rank_ = lu_decomp.rank();

    return rank_;
}

#endif