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

// Publisher
std_msgs::Float32MultiArray angle;

// Subscriber
std_msgs::Float32MultiArray angular_velocity;

void angular_velocity_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    angular_velocity = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExC_Motor");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float32MultiArray>("angle", 100);
    angle.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber angular_velocity_sub = nh.subscribe<std_msgs::Float32MultiArray>("angular_velocity", 100, angular_velocity_cb);
    angular_velocity.data.resize(JOINT_NUMBER);

    while(nh.ok())
    {
        std::cout

        << std::endl
        << "velocity"
        << std::endl
        << angular_velocity
        << std::endl;

        angle_pub.publish(angle);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}