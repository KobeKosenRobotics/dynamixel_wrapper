/**
* @file position_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2022.9.12
* @details 
*/

#include <ros/ros.h>

#include <iostream>
#include <string>

#include <dynamixel_wrapper/dynamixel_wrapper.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "position_node");
    ros::NodeHandle n;

    double rate=10.0;
    ros::Rate loop_rate(rate);
    
    //dynamixel hub setting
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);

    //dynamixel motor setting
    int motor_id=0;
    int operating_mode=5;
    dynamixel_wrapper::dynamixel_wrapper motor0(motor_id,dxl_base,dynamixel_wrapper::XM430_W350_R,operating_mode);

    //setting
    motor0.setTorqueEnable(false);
    motor0.setCurrentLimit(40.0);
    motor0.setTorqueEnable(true);

    double goal_angle=90.0;

    while (n.ok())  {
        motor0.setGoalPosition(goal_angle);

        if(std::abs(motor0.getPresentPosition()-goal_angle)<1.0){
            goal_angle*=-1;
        }
        ROS_INFO("Motor angle %lf[deg]",motor0.getPresentPosition());
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    return 0;
}