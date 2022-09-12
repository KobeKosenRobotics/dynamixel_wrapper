/**
* @file position_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2022.9.12
* @details 
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <dynamixel_wrapper/dynamixel_wrapper.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    double rate=10.0;
    //制御周期10Hz
    ros::Rate loop_rate(rate);

    //param setting
    ros::NodeHandle pn("~");
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    int motor_id=0;
    int operating_mode=5;
    dynamixel_wrapper::dynamixel_wrapper motor0(motor_id,dxl_base,dynamixel_wrapper::XM430_W350_R,operating_mode);


    motor0.setTorqueEnable(false);


    motor0.setCurrentLimit(40.0);

    std::vector<double> offset_angle={269.121094, 338.027344, 69.609375, 26.894531};
    std::vector<double> max_angle={-200, 231.451172, 138.285156, 44.3};

    while (n.ok())  {
        motor0.setGoalPosition(200.0*(-joy_msg.axes[1])+offset_angle[0]);


        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}