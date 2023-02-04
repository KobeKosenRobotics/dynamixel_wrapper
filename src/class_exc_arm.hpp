#ifndef CLASS_EXC_ARM_HPP
#define CLASS_EXC_ARM_HPP

#include "class_exc_arm_property.hpp"
#include "class_exc_joint.hpp"

// #include "class_quaternion.hpp"

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
    private:
        // Joint
        ExCJoint exc_joint[JOINT_NUMBER];

        // Bool
        bool _motor_enable = false;
        bool _emergency_stop = false;

        // Calculation Mode 0(else):Angle, 1:TimeDiff, 2:ExC
        int _calculation_mode = 0, _calculation_mode_old = 0;

        // Motor
        Eigen::Matrix<double, JOINT_NUMBER, 1> _sensor_angle, _motor_angular_velocity;

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, 1> _target_angle;

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> _pose;
        Eigen::Matrix<double, 3, 1> _position, _euler;
        Eigen::Matrix<double, 3, 3> _rotation_all;

        // Inverse Kinematics
        double _proportional_gain = 20.0;
        Eigen::Matrix<double, 6, 1> _target_pose;

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 50;    // _liner_velocity[mm/s]

        // TimeDiff: Time Differentiation
        Eigen::Matrix<double, 6, 6> _time_diff_jacobian;
        Eigen::Matrix<double, 3, 6> _translation_jacobian, _rotation_jacobian;
        Eigen::Matrix<double, 3, 3> _alternating_euler;
        Eigen::Matrix<double, 3, 6> _alternating_rotation;
        bool _is_first_replace_variables = true;
        double lofx, lofy, lofz, c12, s12,
        l0x, l0y, l0z, t0, c0, s0,
        l1x, l1y, l1z, t1, c1, s1,
        l2x, l2y, l2z, t2, c2, s2,
        l3x, l3y, l3z, t3, c3, s3,
        l4x, l4y, l4z, t4, c4, s4,
        l5x, l5y, l5z, t5, c5, s5;

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6, JOINT_NUMBER> _exc_jacobian;
        Eigen::Matrix<double, 6, JOINT_NUMBER> _exc_jacobian_body;
        Eigen::Matrix<double, 6, 6> _transformation_matrix;
        Eigen::Matrix<double, 3, 3> _transformation_euler;

        // Other
        Eigen::Matrix<double, 3, 3> _zero;

        // Experimentation
        double _total_calculation_time = 0.0;
        Eigen::Matrix<double, 6, 1> _pose_error;
        int _test_number = 0;
        ros::Time _calculation_time_start, _calculation_time_end;
        int _calculation_number = 0;

    public:
        // Constructor
        ExCArm();

        // Debug
        void print();

        // Experimentation
        void setCalculationMode(int calculation_mode_);
        void setTargetAngle(double angle0_, double angle1_, double angle2_, double angle3_, double angle4_, double angle5_);
        void setTargetPose(double x_, double y_, double z_, double ez_, double ey_, double ex_);
        bool isInTargetAngle();
        bool isInTargetPose();
        void measurementStart();
        void measurementEnd();
        double getDurationTIme();

        // Subscribe
        void setMotorEnable(std_msgs::Bool motor_enable);
        void setEmergencyStop(std_msgs::Bool emergency_stop);
        void setCalculationMode(std_msgs::Int16 calculation_mode);
        void setSensorAngle(std_msgs::Float32MultiArray sensor_angle);
        void setTargetAngle(std_msgs::Float32MultiArray target_angle);
        void setTargetPose(geometry_msgs::Pose target_pose);
            void setTargetPoseStart();
            double getDistance();

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> getPose();
            Eigen::Matrix<double, 3, 1> getPosition();
            Eigen::Matrix<double, 3, 1> getEuler();

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> getMidTargetPoseLinearInterpolation();

        // Publish
        bool getMotorEnable();
        bool getEmergencyStop();
        int getCalculationMode();
        std_msgs::Float32MultiArray getMotorAngularVelocityZero();
        std_msgs::Float32MultiArray getMotorAngularVelocity();
            void changeMotorAngularVelocity();
                void setMotorAngularVelocityZero();
                void getMotorAngularVelocityByAngle();
                void getMotorAngularVelocityByTimeDiff();
                void getMotorAngularVelocityByExC();

        // TimeDiff: Time Differentiation
        Eigen::Matrix<double, 6, 6> getTimeDiffJacobian();
            Eigen::Matrix<double, 3, 6> getTranslationJacobian();
            Eigen::Matrix<double, 3, 6> getRotationJacobian();
            void replaceVariables();

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobian();
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobianBody();
            Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 6, 6> adjointInverse(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> vector);
        Eigen::Matrix<double, 6, 6> getTransformationMatrix();
            Eigen::Matrix<double, 3, 3> getTransformationEuler();
};

// Constructor
ExCArm::ExCArm()
{
    _zero <<
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0;

    // ExC Joint Set Joint
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        exc_joint[i].setJoint(i);
    }
}

// Debug
void ExCArm::print()
{
    std::cout

    // << std::endl
    // << "target angle"
    // << std::endl
    // << _target_angle

    // << std::endl
    // << "sensor angle"
    // << std::endl
    // << _sensor_angle

    << std::endl
    << "pose"
    << std::endl
    << getPose()

    // << std::endl
    // << "time diff jacobian"
    // << std::endl
    // << getTimeDiffJacobian()

    // << std::endl
    // << "exc jacobian"
    // << std::endl
    // << getExCJacobian()

    // << std::endl
    // << "mid target pose"
    // << std::endl
    // << getMidTargetPoseLinearInterpolation()

    // << std::endl
    // << "motor angular velocity"
    // << std::endl
    // << _motor_angular_velocity

    // << std::endl
    // << "exc jacobian"
    // << std::endl
    // << getExCJacobian()

    << std::endl;
}

// Experimentation
void ExCArm::setCalculationMode(int calculation_mode_)
{
    _calculation_mode = calculation_mode_;
}

void ExCArm::setTargetAngle(double angle0_, double angle1_, double angle2_, double angle3_, double angle4_, double angle5_)
{
    _target_angle(0,0) = angle0_;
    _target_angle(1,0) = angle1_;
    _target_angle(2,0) = angle2_;
    _target_angle(3,0) = angle3_;
    _target_angle(4,0) = angle4_;
    _target_angle(5,0) = angle5_;
}

void ExCArm::setTargetPose(double x_, double y_, double z_, double ez_, double ey_, double ex_)
{
    _target_pose(0,0) = x_;
    _target_pose(1,0) = y_;
    _target_pose(2,0) = z_;
    _target_pose(3,0) = ez_;
    _target_pose(4,0) = ey_;
    _target_pose(5,0) = ex_;
    setTargetPoseStart();
}

bool ExCArm::isInTargetAngle()
{
    Eigen::Matrix<double, JOINT_NUMBER, 1> tolerance_angle_;
    tolerance_angle_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        if(fabs(_target_angle(i,0) - _sensor_angle(i,0)) > tolerance_angle_(i,0)) return false;
    }
    return true;
}

bool ExCArm::isInTargetPose()
{
    Eigen::Matrix<double, 6, 1> tolerance_pose_;
    tolerance_pose_ << 0.01, 0.01, 0.01, 0.001, 0.001, 0.001;
    for(int i = 0; i < 6; i++)
    {
        if(fabs(_target_pose(i,0) - _pose(i,0)) > tolerance_pose_(i,0)) return false;
    }
    return true;
}

void ExCArm::measurementStart()
{
    _total_calculation_time = 0.0;
    _calculation_number = 0;
}

void ExCArm::measurementEnd()
{
    _pose_error = _target_pose - _pose;

    std::cout
    << _calculation_mode << ", "
    << _test_number << ", "
    << _total_calculation_time << ", "
    << _calculation_number << ", "
    << _total_calculation_time/_calculation_number << ", "
    << _pose_error(0,0) << ", "
    << _pose_error(1,0) << ", "
    << _pose_error(2,0) << ", "
    << _pose_error(3,0) << ", "
    << _pose_error(4,0) << ", "
    << _pose_error(5,0) << ","
    << std::endl;

    _test_number++;
}
double ExCArm::getDurationTIme()
{
    return _duration_time;
}

// Subscribe
void ExCArm::setMotorEnable(std_msgs::Bool motor_enable)
{
    _motor_enable = motor_enable.data;
}

void ExCArm::setEmergencyStop(std_msgs::Bool emergency_stop)
{
    _emergency_stop = emergency_stop.data;
}

void ExCArm::setCalculationMode(std_msgs::Int16 calculation_mode)
{
    _calculation_mode_old = _calculation_mode;
    _calculation_mode = calculation_mode.data;
}

void ExCArm::setSensorAngle(std_msgs::Float32MultiArray sensor_angle)
{
    sensor_angle.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _sensor_angle(i,0) = sensor_angle.data[i];
    }
}

void ExCArm::setTargetAngle(std_msgs::Float32MultiArray target_angle)
{
    target_angle.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _target_angle(i,0) = target_angle.data[i];
    }
}

void ExCArm::setTargetPose(geometry_msgs::Pose target_pose)
{
    if((target_pose != _target_pose_old) || (_calculation_mode != _calculation_mode_old))
    {
        _target_pose(0,0) = target_pose.position.x;
        _target_pose(1,0) = target_pose.position.y;
        _target_pose(2,0) = target_pose.position.z;
        _target_pose(3,0) = target_pose.orientation.z;
        _target_pose(4,0) = target_pose.orientation.y;
        _target_pose(5,0) = target_pose.orientation.x;
        setTargetPoseStart();
    }

    _target_pose_old = target_pose;
}

void ExCArm::setTargetPoseStart()
{
    _target_pose_start = _pose;
    _time_start_move = ros::Time::now();
    _duration_time = getDistance()/_linear_velocity;
}

double ExCArm::getDistance()
{
    return sqrt(pow((_target_pose(0,0)-_target_pose_start(0,0)),2)+pow((_target_pose(1,0)-_target_pose_start(1,0)),2)+pow((_target_pose(2,0)-_target_pose_start(2,0)),2));
}

// Forward Kinematics
Eigen::Matrix<double, 6, 1> ExCArm::getPose()
{
    getPosition();
    getEuler();
    _pose(0,0) = _position(0,0);
    _pose(1,0) = _position(1,0);
    _pose(2,0) = _position(2,0);
    _pose(3,0) = _euler(0,0);
    _pose(4,0) = _euler(1,0);
    _pose(5,0) = _euler(2,0);

    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Pose
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "pose";
    transformStamped.transform.translation.x = _pose(0,0)/1000.0;
    transformStamped.transform.translation.y = _pose(1,0)/1000.0;
    transformStamped.transform.translation.z = _pose(2,0)/1000.0;

    q.setRPY(_euler(2,0), _euler(1,0), _euler(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    return _pose;
}

Eigen::Matrix<double, 3, 1> ExCArm::getPosition()
{
    _position = exc_arm_property.getLink(JOINT_NUMBER);
    for(int i = JOINT_NUMBER-1; 0 <= i; i--)
    {
        _position = exc_arm_property.getLink(i) + exc_arm_property.getRotationMatrix(i, _sensor_angle(i,0))*_position;
    }

    return _position;
}

Eigen::Matrix<double, 3, 1> ExCArm::getEuler()
{
    _rotation_all = exc_arm_property.getRotationMatrix(JOINT_NUMBER, 0.0);
    for(int i = JOINT_NUMBER-1; i >= 0; i--)
    {
        _rotation_all = exc_arm_property.getRotationMatrix(i, _sensor_angle(i,0))*_rotation_all;
    }

    // ZYX Euler
    _euler(1,0) = -asin(_rotation_all(2,0));
    _euler(0,0) = acos(_rotation_all(0,0)/cos(_euler(1,0)));
    if(_rotation_all(1,0)/cos(_euler(1,0)) < 0) _euler(0,0) *= (-1);
    _euler(2,0) = acos(_rotation_all(2,2)/cos(_euler(1,0)));
    if(_rotation_all(2,1)/cos(_euler(1,0)) < 0) _euler(2,0) *= (-1);

    for(int i = 0; i < 3; i++)
    {
        if(isnan(_euler(i,0))) _euler(i,0) = 0.0;
    }

    return _euler;
}

// Linear Interpolation
Eigen::Matrix<double, 6, 1> ExCArm::getMidTargetPoseLinearInterpolation()
{
    _midpoint = std::min(std::max((ros::Time::now()-_time_start_move).toSec()/_duration_time, 0.0), 1.0);
    return _midpoint*_target_pose +(1-_midpoint)*_target_pose_start;
}

// Publish
bool ExCArm::getMotorEnable()
{
    return _motor_enable;
}

bool ExCArm::getEmergencyStop()
{
    return _emergency_stop;
}

int ExCArm::getCalculationMode()
{
    return _calculation_mode;
}

std_msgs::Float32MultiArray ExCArm::getMotorAngularVelocityZero()
{
    std_msgs::Float32MultiArray motor_angular_velocity_zero_;
    motor_angular_velocity_zero_.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        motor_angular_velocity_zero_.data[i] = 0.0;
    }

    return motor_angular_velocity_zero_;
}

std_msgs::Float32MultiArray ExCArm::getMotorAngularVelocity()
{
    changeMotorAngularVelocity();

    std_msgs::Float32MultiArray motor_angular_velocity_;
    motor_angular_velocity_.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        motor_angular_velocity_.data[i] = _motor_angular_velocity(i,0);
    }

    return motor_angular_velocity_;
}

void ExCArm::changeMotorAngularVelocity()
{
    if(_emergency_stop)
    {
        setMotorAngularVelocityZero();
        return;
    }

    if(_calculation_mode == 1)
    {
        _calculation_number++;
        _calculation_time_start = ros::Time::now();
        getMotorAngularVelocityByTimeDiff();
        _calculation_time_end = ros::Time::now();
        _total_calculation_time += (_calculation_time_end - _calculation_time_start).toSec();
        return;
    }
    else if(_calculation_mode == 2)
    {
        _calculation_number++;
        _calculation_time_start = ros::Time::now();
        getMotorAngularVelocityByExC();
        _calculation_time_end = ros::Time::now();
        _total_calculation_time += (_calculation_time_end - _calculation_time_start).toSec();
        return;
    }

    else
    {
        getMotorAngularVelocityByAngle();
        return;
    }
}

void ExCArm::setMotorAngularVelocityZero()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _motor_angular_velocity(i,0) = 0.0;
    }
}

void ExCArm::getMotorAngularVelocityByAngle()
{
    getPose();
    _motor_angular_velocity = exc_arm_property.getProportionalGainAngleOperating()*(_target_angle - _sensor_angle);
}

void ExCArm::getMotorAngularVelocityByTimeDiff()
{
    #ifdef DOF6
    _motor_angular_velocity = _proportional_gain*(getTimeDiffJacobian().inverse())*(getMidTargetPoseLinearInterpolation()-getPose());
    #endif

    #ifndef DOF6
    std::cout << "error: JOINT_NUMBER is not 6" << std::endl << "Time Differentiation Jacobian can only be used for models with JOINT_NUMBER == 6" << std::endl;
    #endif
}

void ExCArm::getMotorAngularVelocityByExC()
{
    #ifdef DOF6
    _motor_angular_velocity = _proportional_gain*(getExCJacobian().inverse())*(getMidTargetPoseLinearInterpolation()-getPose());
    #endif

    #ifndef DOF6
    _motor_angular_velocity = _proportional_gain*(exc_arm_property.getPseudoInverseMatrix(getExCJacobian()))*(getMidTargetPoseLinearInterpolation()-getPose());
    #endif
}

// TimeDiff; Time Differentiation
Eigen::Matrix<double, 6, 6> ExCArm::getTimeDiffJacobian()
{
    replaceVariables();

    _translation_jacobian = getTranslationJacobian();
    _rotation_jacobian = getRotationJacobian();

    _time_diff_jacobian << _translation_jacobian, _rotation_jacobian;

    return _time_diff_jacobian;
}

Eigen::Matrix<double, 3, 6> ExCArm::getTranslationJacobian()
{
    _translation_jacobian(0,0) = - s0*(l0x + c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) + s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))))) - c0*(l0y + l1y + l2y + c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(0,1) = c0*(c1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(0,2) = c0*(c1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(0,3) = s0*(s3*(l3y + l4y + l5y*c5 + l5x*s5) - c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))) - c12*c0*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(0,4) = s0*s3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)) - c0*(c1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) + s1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))));
    _translation_jacobian(0,5) = - c0*(c1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5)) - s1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5))) - s0*(c3*(l5x*c5 - l5y*s5) - c4*s3*(l5y*c5 + l5x*s5));

    _translation_jacobian(1,0) = c0*(l0x + c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) + s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))))) - s0*(l0y + l1y + l2y + c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(1,1) = s0*(c1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(1,2) = s0*(c1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))));
    _translation_jacobian(1,3) = - c0*(s3*(l3y + l4y + l5y*c5 + l5x*s5) - c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))) - c12*s0*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(1,4) = - s0*(c1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) + s1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)))) - c0*s3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z));
    _translation_jacobian(1,5) = c0*(c3*(l5x*c5 - l5y*s5) - c4*s3*(l5y*c5 + l5x*s5)) - s0*(c1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5)) - s1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5)));

    _translation_jacobian(2,0) = 0.0;
    _translation_jacobian(2,1) = - c1*(l1x + s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(l1z + c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))));
    _translation_jacobian(2,2) = - c1*(s2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) + c2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)))) - s1*(c2*(l2z + l3z - s4*(l4x + l5x*c5 - l5y*s5) + c4*(l4z + l5z)) - s2*(l2x - s3*(l3y + l4y + l5y*c5 + l5x*s5) + c3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z))));
    _translation_jacobian(2,3) = s12*(c3*(l3y + l4y + l5y*c5 + l5x*s5) + s3*(l3x + c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)));
    _translation_jacobian(2,4) = s1*(s2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) + c2*c3*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z))) - c1*(c2*(c4*(l4x + l5x*c5 - l5y*s5) + s4*(l4z + l5z)) - c3*s2*(s4*(l4x + l5x*c5 - l5y*s5) - c4*(l4z + l5z)));
    _translation_jacobian(2,5) = c1*(s2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) + c2*s4*(l5y*c5 + l5x*s5)) + s1*(c2*(s3*(l5x*c5 - l5y*s5) + c3*c4*(l5y*c5 + l5x*s5)) - s2*s4*(l5y*c5 + l5x*s5));

    return _translation_jacobian;
}

Eigen::Matrix<double, 3, 6> ExCArm::getRotationJacobian()
{
    _alternating_euler <<
    0.0, -sin(_euler(0,0)), cos(_euler(1,0))*cos(_euler(0,0)),
    0.0,  cos(_euler(0,0)), cos(_euler(1,0))*sin(_euler(0,0)),
    1.0,               0.0,                 -sin(_euler(1,0));

    _alternating_rotation <<
    0.0, -s0, -s0, s12*c1,  c0*s1*s2*s3 -c0*c1*c2*s3 - c3*s0,    c0*c1*c4*s2 - s0*s3*s4 +c0*c2*c4*s1 +c0*c1*c2*c3*s4 -c0*c3*s1*s2*s4,
    0.0,  c0,  c0, s12*s0, c0*c3 - c1*c2*s0*s3 + s0*s1*s2*s3, c0*s3*s4 + c1*c4*s0*s2 + c2*c4*s0*s1 + c1*c2*c3*s0*s4 - c3*s0*s1*s2*s4,
    1.0, 0.0, 0.0,    c12,                            s12*s3,                        c1*c2*c4 - c4*s1*s2 - c1*c3*s2*s4 - c2*c3*s1*s4;

    _rotation_jacobian = _alternating_euler.inverse()*_alternating_rotation;
    return _rotation_jacobian;
}

void ExCArm::replaceVariables()
{
    if(_is_first_replace_variables)
    {
        lofx = exc_arm_property.getLink(0, 0);
        lofy = exc_arm_property.getLink(0, 1);
        lofz = exc_arm_property.getLink(0, 2);

        l0x =exc_arm_property.getLink(1,0);
        l0y =exc_arm_property.getLink(1,1);
        l0z =exc_arm_property.getLink(1,2);

        l1x =exc_arm_property.getLink(2,0);
        l1y =exc_arm_property.getLink(2,1);
        l1z =exc_arm_property.getLink(2,2);

        l2x =exc_arm_property.getLink(3,0);
        l2y =exc_arm_property.getLink(3,1);
        l2z =exc_arm_property.getLink(3,2);

        l3x =exc_arm_property.getLink(4,0);
        l3y =exc_arm_property.getLink(4,1);
        l3z =exc_arm_property.getLink(4,2);

        l4x =exc_arm_property.getLink(5,0);
        l4y =exc_arm_property.getLink(5,1);
        l4z =exc_arm_property.getLink(5,2);

        l5x =exc_arm_property.getLink(6,0);
        l5y =exc_arm_property.getLink(6,1);
        l5z =exc_arm_property.getLink(6,2);

        _is_first_replace_variables = false;
    }

    t0 = _sensor_angle(0,0);
    t1 = _sensor_angle(1,0);
    t2 = _sensor_angle(2,0);
    t3 = _sensor_angle(3,0);
    t4 = _sensor_angle(4,0);
    t5 = _sensor_angle(5,0);

    c0 = cos(t0);
    c1 = cos(t1);
    c2 = cos(t2);
    c3 = cos(t3);
    c4 = cos(t4);
    c5 = cos(t5);
    c12 = cos(t1+t2);

    s0 = sin(t0);
    s1 = sin(t1);
    s2 = sin(t2);
    s3 = sin(t3);
    s4 = sin(t4);
    s5 = sin(t5);
    s12 = sin(t1+t2);
}

// ExC: Exponential Coordinates
Eigen::Matrix<double, 6, JOINT_NUMBER> ExCArm::getExCJacobian()
{
    _exc_jacobian = (getTransformationMatrix().inverse())*getExCJacobianBody();

    return _exc_jacobian;
}

Eigen::Matrix<double, 6, JOINT_NUMBER> ExCArm::getExCJacobianBody()
{
    Eigen::Matrix<double, 6, 1> xi_dagger_[JOINT_NUMBER];

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        Eigen::Matrix<double, 4, 4> matrix_;
        matrix_ = exc_arm_property.getGstZero();

        for(int j = JOINT_NUMBER-1; i <= j; j--)
        {
            matrix_ = exc_joint[j].getExpXiHatTheta(_sensor_angle(j,0))*matrix_;
        }

        xi_dagger_[i] = adjointInverse(matrix_)*exc_joint[i].getXi();

        for(int k = 0; k < 6; k++)
        {
            _exc_jacobian_body(k,i) = xi_dagger_[i](k,0);
        }
    }

    return _exc_jacobian_body;
}

Eigen::Matrix<double, 6, 6> ExCArm::adjoint(Eigen::Matrix<double, 4, 4> matrix)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_matrix_;

    rotation_matrix_ <<
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2);

    position_ <<
    matrix(0,3),
    matrix(1,3),
    matrix(2,3);

    adjoint_matrix_ <<
    rotation_matrix_, hat(position_)*rotation_matrix_,
               _zero,                rotation_matrix_;

    return adjoint_matrix_;
}

Eigen::Matrix<double, 6, 6> ExCArm::adjointInverse(Eigen::Matrix<double, 4, 4> matrix)
{
    Eigen::Matrix<double, 3, 3> rotation_matrix_;
    Eigen::Matrix<double, 3, 1> position_;
    Eigen::Matrix<double, 6, 6> adjoint_inverse_matrix_;

    rotation_matrix_ <<
    matrix(0,0), matrix(0,1), matrix(0,2),
    matrix(1,0), matrix(1,1), matrix(1,2),
    matrix(2,0), matrix(2,1), matrix(2,2);

    position_ <<
    matrix(0,3),
    matrix(1,3),
    matrix(2,3);

    adjoint_inverse_matrix_ <<
    rotation_matrix_.transpose(), -rotation_matrix_.transpose()*hat(position_),
                           _zero,                 rotation_matrix_.transpose();

    return adjoint_inverse_matrix_;
}

Eigen::Matrix<double, 3, 3> ExCArm::hat(Eigen::Matrix<double, 3, 1> vector)
{
    Eigen::Matrix<double, 3, 3> hat_vector_;

    hat_vector_ <<
             0.0, -vector(2,0),  vector(1,0),
     vector(2,0),          0.0, -vector(0,0),
    -vector(1,0),  vector(0,0),          0.0;

    return hat_vector_;
}

Eigen::Matrix<double, 6, 6> ExCArm::getTransformationMatrix()
{
    getTransformationEuler();

    _transformation_matrix <<
    _rotation_all.transpose(),                 _zero,
                        _zero, _transformation_euler;

    return _transformation_matrix;
}

Eigen::Matrix<double, 3, 3> ExCArm::getTransformationEuler()
{
    _transformation_euler <<
                    -sin(_euler(1,0)),               0.0, 1.0,
    cos(_euler(1,0))*sin(_euler(2,0)),  cos(_euler(2,0)), 0.0,
    cos(_euler(2,0))*cos(_euler(1,0)), -sin(_euler(2,0)), 0.0;

    return _transformation_euler;
}

#endif