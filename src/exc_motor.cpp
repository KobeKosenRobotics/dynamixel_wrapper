// #define NO_MOTOR

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

// Global
double max_motor_angular_velocity = 6.0;    // max_motor_angular_velocity[rad/s]

// Publisher
std_msgs::Float32MultiArray MCA;

// Subscriber
std_msgs::Float32MultiArray CMAV;

// Function
std_msgs::Float32MultiArray changeMotorAngularVelocity(std_msgs::Float32MultiArray msg)
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        if(max_motor_angular_velocity < fabs(msg.data[i])) msg.data[i] = max_motor_angular_velocity*radps2rpm*msg.data[i]/fabs(msg.data[i]);
    }
    return msg;
}

void CMAV_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    CMAV = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExCArmMotor");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher MCA_pub = nh.advertise<std_msgs::Float32MultiArray>("MCA", 100);
    MCA.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber CMAV_sub = nh.subscribe<std_msgs::Float32MultiArray>("CMAV", 100, CMAV_cb);
    CMAV.data.resize(JOINT_NUMBER);

    #ifndef NO_MOTOR
    // Motor Declaration
    std::string port_name("/dev/ttyUSB0");
    int baudrate = 1000000;
    // TODO: motor1, motor2, ..., motor5 -> motor[JOINT_NUMBER]
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor1(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor2(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor3(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor4(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor5(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);

    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);

    motor0.setHomingOffset( 0.0);
    motor1.setHomingOffset(35.0);
    motor2.setHomingOffset(45.0);
    motor3.setHomingOffset( 0.0);
    motor4.setHomingOffset( 0.0);
    motor5.setHomingOffset( 0.0);

    motor0.setTorqueEnable(true);
    motor1.setTorqueEnable(true);
    motor2.setTorqueEnable(true);
    motor3.setTorqueEnable(true);
    motor4.setTorqueEnable(true);
    motor5.setTorqueEnable(true);
    #endif

    while(nh.ok())
    {
        #ifndef NO_MOTOR
        MCA.data[0] = motor0.getPresentPosition()*deg2rad;
        MCA.data[1] = motor1.getPresentPosition()*deg2rad;
        MCA.data[2] = motor2.getPresentPosition()*deg2rad;
        MCA.data[3] = motor3.getPresentPosition()*deg2rad;
        MCA.data[4] = motor4.getPresentPosition()*deg2rad;
        MCA.data[5] = motor5.getPresentPosition()*deg2rad;
        #endif

        CMAV = changeMotorAngularVelocity(CMAV);

        #ifndef NO_MOTOR
        motor0.setGoalVelocity(CMAV.data[0]);
        motor1.setGoalVelocity(CMAV.data[1]);
        motor2.setGoalVelocity(CMAV.data[2]);
        motor3.setGoalVelocity(CMAV.data[3]);
        motor4.setGoalVelocity(CMAV.data[4]);
        motor5.setGoalVelocity(CMAV.data[5]);
        #endif

        MCA_pub.publish(MCA);

        ros::spinOnce();
        loop_rate.sleep();
    }

    #ifndef NO_MOTOR
    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);
    #endif NO_MOTOR

    return 0;
}