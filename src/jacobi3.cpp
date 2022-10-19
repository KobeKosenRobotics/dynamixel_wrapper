#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
 
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
 
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
 
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
 
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

#define SIMULATION

// Global Variables
// Simulation
bool sim_is_first = true;
ros::Time sim_start_time, sim_end_time;
Eigen::Matrix<double, 6, 1> sim_theta;

// 6 x 1
Eigen::Matrix<double, 6, 1> target_pose, now_pose;
// Joint x 1
Eigen::Matrix<double, 6, 1> initial_theta, target_theta, target_theta_dot;

// Arm Property
Eigen::Matrix<double, 3, 1> link_offset, link0, link1, link2, link3, link4, link5;
double l_offset, l0, l1, l2, l3, l4, l5;
Eigen::Matrix<double, 6, 1> homing_offset;

// Motor Declaration
std::string port_name("/dev/ttyUSB0");
int baudrate=1000000;
dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
dynamixel_wrapper::dynamixel_wrapper motor0(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
dynamixel_wrapper::dynamixel_wrapper motor1(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 1);
dynamixel_wrapper::dynamixel_wrapper motor2(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
dynamixel_wrapper::dynamixel_wrapper motor3(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 1);
dynamixel_wrapper::dynamixel_wrapper motor4(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);
dynamixel_wrapper::dynamixel_wrapper motor5(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 1);

// Function
void trash_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    target_pose(0,0) = msg->position.x;
    target_pose(1,0) = msg->position.y;
    target_pose(2,0) = msg->position.z;
    target_pose(3,0) = msg->orientation.x;
    target_pose(4,0) = msg->orientation.y;
    target_pose(5,0) = msg->orientation.z;

    return;
};

void tf_broadcaster()
{
    if(sim_is_first)
    {
        sim_start_time = ros::Time::now();
        sim_theta = homing_offset+initial_theta;

        return;
    }

    sim_end_time = ros::Time::now();
    sim_theta += target_theta_dot*(sim_start_time-sim_end_time).toSec();
    sim_start_time = sim_end_time;

    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Joint 0
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = "joint0";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.159;
    
    q.setRPY(0, 0, sim_theta(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
    
    // Joint 1
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint0";
    transformStamped.child_frame_id = "joint1";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    
    q.setRPY(0, sim_theta(1,0), 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 2
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint1";
    transformStamped.child_frame_id = "joint2";
    transformStamped.transform.translation.x = 0.030;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.264;
    
    q.setRPY(0, sim_theta(2,0), 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 3
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint2";
    transformStamped.child_frame_id = "joint3";
    transformStamped.transform.translation.x = -0.030;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.258;
    
    q.setRPY(0, 0, sim_theta(3,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 4
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint3";
    transformStamped.child_frame_id = "joint4";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    
    q.setRPY(0, sim_theta(4,0), 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 5
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint4";
    transformStamped.child_frame_id = "joint5";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.123;
    
    q.setRPY(0, 0, sim_theta(5,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArmSequence");
    ros::NodeHandle nh;
    double rate = 10.0f;
    ros::Rate loop_rate(rate);

    // Subscriber
    ros::Subscriber trash_pose_sub = nh.subscribe<geometry_msgs::Pose>("trash_pose", 10, trash_pose_cb);
    
    // Arm Property
    link_offset << 0.0, 0.0, 159.0;
    link0 << 0.0, 0.0, 0.0;
    link1 << 30.0, 0.0, 264.0;
    link2 << -30.0, 0.0, 258.0;
    link3 << 0.0, 0.0, 0.0;
    link4 << 0.0, 0.0, 123.0;    // Without End Effector
    link5 << 0.0, 0.0, 0.0;

    l_offset = sqrt(pow(link_offset(0,0),2.0)+pow(link_offset(1,0),2.0)+pow(link_offset(2,0),2.0));
    l0 = sqrt(pow(link0(0,0),2.0)+pow(link0(1,0),2.0)+pow(link0(2,0),2.0));
    l1 = sqrt(pow(link1(0,0),2.0)+pow(link1(1,0),2.0)+pow(link1(2,0),2.0));
    l2 = sqrt(pow(link2(0,0),2.0)+pow(link2(1,0),2.0)+pow(link2(2,0),2.0));
    l3 = sqrt(pow(link3(0,0),2.0)+pow(link3(1,0),2.0)+pow(link3(2,0),2.0));
    l4 = sqrt(pow(link4(0,0),2.0)+pow(link4(1,0),2.0)+pow(link4(2,0),2.0));
    l5 = sqrt(pow(link5(0,0),2.0)+pow(link5(1,0),2.0)+pow(link5(2,0),2.0));

    homing_offset << 0.0, atan2(30.0,264.0), -atan2(30.0,264.0)+atan2(-30.0,258.0), 0.0, -atan2(-30.0,258.0), 0.0;
    homing_offset *= 180.0/M_PI;
    
    // Motor Set Up
    motor0.setTorqueEnable(false);
    motor0.setVelocityLimit(15);
    motor0.setHomingOffset(homing_offset(0,0));
    motor0.setPositionGain(15, 0, 0);
    motor0.setTorqueEnable(true);
    
    motor1.setTorqueEnable(false);
    motor1.setVelocityLimit(15);
    motor1.setHomingOffset(36.3+homing_offset(1,0));
    motor1.setPositionGain(15, 0, 0);
    motor1.setTorqueEnable(true);
    
    motor2.setTorqueEnable(false);
    motor2.setVelocityLimit(15);
    motor2.setHomingOffset(45+homing_offset(2,0));
    motor2.setPositionGain(15, 0, 0);
    motor2.setTorqueEnable(true);
    
    motor3.setTorqueEnable(false);
    motor3.setVelocityLimit(15);
    motor3.setHomingOffset(homing_offset(3,0));
    motor3.setPositionGain(15, 0, 0);
    motor3.setTorqueEnable(true);
    
    motor4.setTorqueEnable(false);
    motor4.setVelocityLimit(15);
    motor4.setHomingOffset(homing_offset(4,0));
    motor4.setPositionGain(15, 0, 0);
    motor4.setTorqueEnable(true);
    
    motor5.setTorqueEnable(false);
    motor5.setVelocityLimit(15);
    motor5.setHomingOffset(homing_offset(5,0));
    motor3.setPositionGain(15, 0, 0);
    motor5.setTorqueEnable(true);

    homing_offset *= M_PI/180.0;

    while(nh.ok())
    {
        target_theta_dot = target_pose;
        tf_broadcaster();
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Torque Off
    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);

    return 0;
}