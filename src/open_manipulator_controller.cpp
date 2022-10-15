/**
* @file open_manipulator_control.cpp
* @brief
* @author Akiro Harada
* @date 2022.8.13
* @details
*/
 
#include <ros/ros.h>
 
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
 
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
 
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>
 
#include <Eigen/Core>
 
geometry_msgs::Pose init_pose;
 
ros::Time start_time;
 
geometry_msgs::Pose now_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose target_pose_old;
double duration_time = 0.0f;
 
ros::Publisher state_pub;
void pose_cb(geometry_msgs::Pose::ConstPtr pose)
{
    if(*pose == target_pose) return;
    target_pose_old = now_pose;
    target_pose = *pose;
    start_time = ros::Time::now();
    duration_time = pose->orientation.w;
    std_msgs::Bool msg;
    msg.data = false;
    state_pub.publish(msg);
}
 
int main(int argc, char **argv){
    
    ros::init(argc, argv, "dynamixel_wrapper_example_node");
    ros::NodeHandle nh;
    
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, pose_cb);
    
    state_pub = nh.advertise<std_msgs::Bool>("open_manipulator_controller/isGoaled", 1);
    
    double rate = 10.0f;
    ros::Rate loop_rate(rate);
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;
    
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 4);
    dynamixel_wrapper::dynamixel_wrapper motor2(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 4);
    dynamixel_wrapper::dynamixel_wrapper motor3(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
    dynamixel_wrapper::dynamixel_wrapper motor4(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
    dynamixel_wrapper::dynamixel_wrapper motor5(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
    dynamixel_wrapper::dynamixel_wrapper motor6(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
    
    motor1.setTorqueEnable(false);
    motor1.setVelocityLimit(15);
    motor1.setHomingOffset(0);
    motor1.setPositionGain(15, 0, 0);
    motor1.setTorqueEnable(true);
    
    motor2.setTorqueEnable(false);
    motor2.setVelocityLimit(15);
    motor2.setHomingOffset(36.3+6.383073693);
    motor2.setPositionGain(15, 0, 0);
    motor2.setTorqueEnable(true);
    
    motor3.setTorqueEnable(false);
    motor3.setVelocityLimit(15);
    motor3.setHomingOffset(45-6.383073693-6.632514615);
    motor3.setPositionGain(15, 0, 0);
    motor3.setTorqueEnable(true);
    
    motor4.setTorqueEnable(false);
    motor4.setVelocityLimit(15);
    motor4.setHomingOffset(0);
    motor4.setPositionGain(15, 0, 0);
    motor4.setTorqueEnable(true);
    
    motor5.setTorqueEnable(false);
    motor5.setVelocityLimit(15);
    motor5.setHomingOffset(6.632514615);
    motor5.setPositionGain(15, 0, 0);
    motor5.setTorqueEnable(true);
    
    motor6.setTorqueEnable(false);
    motor6.setVelocityLimit(15);
    motor6.setHomingOffset(0);
    motor4.setPositionGain(15, 0, 0);
    motor6.setTorqueEnable(true);
    
    int target = 0;
    
    double theta[6] = {0, 0, 0, 0, 0, 0};
    
    bool isValid_old = true;
    
    // Initialize
    init_pose.position.x = 200.0;
    init_pose.position.y = 0.0;
    init_pose.position.z = 100.0;
    init_pose.orientation.x = 0.0;
    init_pose.orientation.y = 90.0;
    init_pose.orientation.z = 0.0;
    init_pose.orientation.w = 5.0;
    
    target_pose = target_pose_old = init_pose;
    
    while(nh.ok())
    {
        // motor1.setGoalPosition(360);
        // ros::spinOnce();
        // continue;
        double theta[6] = {0, 0, 0, 0, 0, 0};
    
        double val = std::min(std::max((ros::Time::now() - start_time).toSec()/duration_time, 0.0), 1.0);
    
        double x,y,z;
        now_pose.position.x = x = target_pose_old.position.x*(1.0-val) + target_pose.position.x*val;
        now_pose.position.y = y = target_pose_old.position.y*(1.0-val) + target_pose.position.y*val;
        now_pose.position.z = z = target_pose_old.position.z*(1.0-val) + target_pose.position.z*val;
        now_pose.orientation.x = target_pose_old.orientation.x*(1.0-val) + target_pose.orientation.x*val;
        now_pose.orientation.y = target_pose_old.orientation.y*(1.0-val) + target_pose.orientation.y*val;
        now_pose.orientation.z = target_pose_old.orientation.z*(1.0-val) + target_pose.orientation.z*val;
    
        // int length = 123 + 125;
        int length = 123;
    
        x -= length*cos(now_pose.orientation.y*M_PI/180.0)*cos(atan2(now_pose.position.y, now_pose.position.x));
        y -= length*cos(now_pose.orientation.y*M_PI/180.0)*sin(atan2(now_pose.position.y, now_pose.position.x));
        z -= length*sin(-now_pose.orientation.y*M_PI/180.0);
    
        double xx = x*x;
        double yy = y*y;
        double zz = z*z;
    
        double r = sqrt(xx+yy);
    
        if(x >= 0)
        {
            theta[0] = atan2(y, x)*180.0f/M_PI;
        }
    
        // theta[1] = 90.0f - atan2(z - 36.0f, r)*180.0f/M_PI - acos((4448.4f+xx+yy+zz-72.0f*z)/(531.4f*sqrt(xx+yy+zz-72.0f*z + 1296.0f))) * 180.0f/M_PI;
        // theta[2] = atan2(r, z - 36.0f) * 180.0/M_PI + acos((-1856.4f + xx + yy + zz - 72.0f * z) / (519.4f * sqrt(xx + yy + zz - 72.0f * z + 1296.0f))) * 180.0f/M_PI - theta[1];
        // theta[3] = 0.0f;
        // theta[4] = 180.0f - theta[1] - theta[2];
        // theta[5] = 0.0f;
    
        theta[1] = 90.0f - atan2(z - 159.0f, r)*180.0f/M_PI - acos((28433.4f+xx+yy+zz-318.0f*z)/(531.4f*sqrt(xx+yy+zz-318.0f*z + 25281.0f))) * 180.0f/M_PI;
        theta[2] = atan2(r, z - 159.0f) * 180.0/M_PI + acos((22128.6f + xx + yy + zz - 318.0f * z) / (519.4f * sqrt(xx + yy + zz - 318.0f * z + 25281.0f))) * 180.0f/M_PI - theta[1];
        theta[3] = now_pose.orientation.x;
        theta[4] = 90.0f - theta[1] - theta[2] + now_pose.orientation.y;
        theta[5] = now_pose.orientation.z;
        
        for(int i=0; i<6; i++)
        {
            std::cout << theta[i] << "    ";
        }
        std::cout << std::endl;

        bool isValid = true;
        for(int i = 0; i < 6; i++)
        {
            if(isnan(theta[i]))
            {
                isValid = false;
                break;
            }
        }
    
        if(isValid)
        {
            motor1.setGoalPosition(theta[0]);
            motor2.setGoalPosition(theta[1]);
            motor3.setGoalPosition(theta[2]);
            motor4.setGoalPosition(theta[3]);
            motor5.setGoalPosition(theta[4]);
            motor6.setGoalPosition(theta[5]);
        }
    
        if(isValid != isValid_old)
        {
            motor1.setLED(isValid?0:255, isValid?255:0, 0);
            motor2.setLED(isValid?0:255, isValid?255:0, 0);
            motor3.setLED(isValid?0:255, isValid?255:0, 0);
            motor4.setLED(isValid?0:255, isValid?255:0, 0);
            motor5.setLED(isValid?0:255, isValid?255:0, 0);
            motor6.setLED(isValid?0:255, isValid?255:0, 0);
        }
    
        std_msgs::Bool msg;
        if(motor1.getIsInPosition() && motor2.getIsInPosition() && motor3.getIsInPosition() && motor4.getIsInPosition() && motor5.getIsInPosition() && motor6.getIsInPosition())
        {
            msg.data = true;
        }
        else
        {
            msg.data = false;
        }
        state_pub.publish(msg);
    
        ros::spinOnce();
        loop_rate.sleep();
    
        isValid_old = isValid;
        }
        motor1.setTorqueEnable(false);
        motor2.setTorqueEnable(false);
        motor3.setTorqueEnable(false);
        motor4.setTorqueEnable(false);
        motor5.setTorqueEnable(false);
        motor6.setTorqueEnable(false);
        return 0;
}
