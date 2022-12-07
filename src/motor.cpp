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

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

Arm arm;

// Publisher
geometry_msgs::Pose angle_pose;
// Subscriber
geometry_msgs::Pose angular_velocity_pose;

void angular_velocity_cb(geometry_msgs::Pose::ConstPtr msg)
{
    angular_velocity_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Motor");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher angle_pub = nh.advertise<geometry_msgs::Pose>("angle", 100);
    
    // Subscriber
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("angular_velocity", 100, angular_velocity_cb);
    
    // Motor Declaration
    std::string port_name("/dev/ttyUSB0");
    int baudrate = 1000000;
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor1(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor2(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor3(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor4(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor5(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);

    motor0.setTorqueEnable(false);
    motor0.setHomingOffset(0.0);

    motor1.setTorqueEnable(false);
    motor1.setHomingOffset(36.3);

    motor2.setTorqueEnable(false);
    motor2.setHomingOffset(45.0);

    motor3.setTorqueEnable(false);
    motor3.setHomingOffset(0.0);

    motor4.setTorqueEnable(false);
    motor4.setHomingOffset(0.0);

    motor5.setTorqueEnable(false);
    motor5.setHomingOffset(0.0);

    // motor0.setTorqueEnable(true);
    // motor1.setTorqueEnable(true);
    // motor2.setTorqueEnable(true);
    // motor3.setTorqueEnable(true);
    // motor4.setTorqueEnable(true);
    // motor5.setTorqueEnable(true);

    while(nh.ok())
    {
        angle_pose.position.x = motor0.getPresentPosition()*deg2rad;
        angle_pose.position.y = motor1.getPresentPosition()*deg2rad;
        angle_pose.position.z = motor2.getPresentPosition()*deg2rad;
        angle_pose.orientation.x = motor3.getPresentPosition()*deg2rad;
        angle_pose.orientation.y = motor4.getPresentPosition()*deg2rad;
        angle_pose.orientation.z = motor5.getPresentPosition()*deg2rad;

        std::cout << angle_pose << std::endl << std::endl;

        // if(fabs(angular_velocity_pose.position.x) < 3.0 && fabs(angle_pose.position.x) < 2.5) motor0.setGoalVelocity(angular_velocity_pose.position.x*radps2rpm);
        // if(fabs(angular_velocity_pose.position.y) < 3.0 && fabs(angle_pose.position.y) < 2.5) motor1.setGoalVelocity(angular_velocity_pose.position.y*radps2rpm);
        // if(fabs(angular_velocity_pose.position.z) < 3.0 && fabs(angle_pose.position.z) < 2.5) motor2.setGoalVelocity(angular_velocity_pose.position.z*radps2rpm);
        // if(fabs(angular_velocity_pose.orientation.x) < 3.0 && fabs(angle_pose.orientation.x) < 2.5) motor3.setGoalVelocity(angular_velocity_pose.orientation.x*radps2rpm);
        // if(fabs(angular_velocity_pose.orientation.y) < 3.0 && fabs(angle_pose.orientation.y) < 2.5) motor4.setGoalVelocity(angular_velocity_pose.orientation.y*radps2rpm);
        // if(fabs(angular_velocity_pose.orientation.z) < 3.0 && fabs(angle_pose.orientation.z) < 2.5) motor5.setGoalVelocity(angular_velocity_pose.orientation.z*radps2rpm);
        
        angle_pub.publish(angle_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);

    return 0;
}