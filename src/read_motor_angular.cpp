#include <ros/ros.h>
 
#include <iostream>
 
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>
 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "dynamixel_wrapper_example_node");
   ros::NodeHandle nh;
 
   std::string port_name("/dev/ttyUSB0");
   double baudrate = 1000000;
 
   dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
   dynamixel_wrapper::dynamixel_wrapper motor(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
 
   motor.setTorqueEnable(false);
   motor.setVelocityLimit(15);
   motor.setHomingOffset(0);
   motor.setPositionGain(15, 0, 0);
   // motor.setVelocityGain(1,0);
   motor.setTorqueEnable(true);
 
   while(nh.ok())
   {
       std::cout << motor.getPresentPosition() << std::endl;
       motor.setGoalPosition(-20);
       ros::spinOnce();
   }
 
   motor.setTorqueEnable(false);
 
   return 0;
}


// #include <ros/ros.h>

// #include <std_msgs/Bool.h>
// #include <geometry_msgs/Pose.h>

// #include <iostream>
// #include <string>
// #include <fstream>
// #include <sstream>

// #include <Eigen/Core>
// #include <Eigen/LU>
// #include <Eigen/Dense>

// #include <dynamixel_sdk/dynamixel_sdk.h>
// #include <dynamixel_wrapper/dynamixel_wrapper.h>

// int main(int argc, char **argv)
// {
//     // Setup
//     ros::init(argc, argv, "dynamixel_wrapper_example_node");
//     ros::NodeHandle nh;

//     double rate = 10.0f;
//     ros::Rate loop_rate(rate);

//     std::string port_name("/dev/ttyUSB0");
//     int baudrate=1000000;

//     // Motor
//     dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
//     dynamixel_wrapper::dynamixel_wrapper motor(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 4);
//     motor.setTorqueEnable(false);
//     motor.setVelocityLimit(15);
//     motor.setHomingOffset(0);
//     motor.setPositionGain(1000, 0, 0);
//     motor.setVelocityGain(1, 0);
//     motor.setTorqueEnable(true);

//     // dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 3);
//     // dynamixel_wrapper::dynamixel_wrapper motor2(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 3);
//     // dynamixel_wrapper::dynamixel_wrapper motor3(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 3);
//     // dynamixel_wrapper::dynamixel_wrapper motor4(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 3);
//     // dynamixel_wrapper::dynamixel_wrapper motor5(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 3);
//     // dynamixel_wrapper::dynamixel_wrapper motor6(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 3);

//     // motor1.setTorqueEnable(false);
//     // motor1.setVelocityLimit(15);
//     // motor1.setHomingOffset(0);
//     // motor1.setPositionGain(15, 0, 0);
//     // motor1.setVelocityGain(1, 0);
//     // // motor1.setTorqueEnable(true);

//     // motor2.setTorqueEnable(false);
//     // motor2.setVelocityLimit(15);
//     // motor2.setHomingOffset(34);
//     // motor2.setPositionGain(15, 0, 0);
//     // motor2.setVelocityGain(1, 0);
//     // motor2.setTorqueEnable(true);

//     // motor3.setTorqueEnable(false);
//     // motor3.setVelocityLimit(15);
//     // motor3.setHomingOffset(45);
//     // motor3.setPositionGain(15, 0, 0);
//     // motor3.setVelocityGain(1, 0);
//     // // motor3.setTorqueEnable(true);

//     // motor4.setTorqueEnable(false);
//     // motor4.setVelocityLimit(15);
//     // motor4.setHomingOffset(0);
//     // motor4.setPositionGain(15, 0, 0);
//     // motor4.setVelocityGain(1, 0);
//     // // motor4.setTorqueEnable(true);

//     // motor5.setTorqueEnable(false);
//     // motor5.setVelocityLimit(15);
//     // motor5.setHomingOffset(0);
//     // motor5.setPositionGain(15, 0, 0);
//     // motor5.setVelocityGain(1, 0);
//     // // motor5.setTorqueEnable(true);

//     // motor6.setTorqueEnable(false);
//     // motor6.setVelocityLimit(15);
//     // motor6.setHomingOffset(0);
//     // motor6.setPositionGain(15, 0, 0);
//     // motor6.setVelocityGain(1, 0);
//     // // motor6.setTorqueEnable(true);
//     // Setup

//     double t1, t2, t3, t4, t5, t6;
//     //Eigen::Matrix<double, 6, 1> theta;

//     while(nh.ok())
//     {
//         //std::cout << motor.read(dynamixel_wrapper::item(48, 1)) << std::endl;
//         //motor.setGoalPosition(600);
//         motor.write(dynamixel_wrapper::item(596, 4), 250960);

//         uint errorNum = motor.read(dynamixel_wrapper::item(48, 1));
//         for(int i=0;i<8;i++)
//         {
//             if((errorNum >> i)&0x01)std::cout << i << std::endl;
//         }
//         // Theta
//         // t1 = motor1.getPresentPosition();
//         // t2 = motor2.getPresentPosition(),
//         // t3 = motor3.getPresentPosition(),
//         // t4 = motor4.getPresentPosition(),
//         // t5 = motor5.getPresentPosition(),
//         // t6 = motor6.getPresentPosition();

//         // theta << t1, t2, t3, t4, t5, t6; 
//         //float theta = motor.getPresentPosition();
//         //std::cout << "theta:" << std::endl << theta << std::endl << std::endl;
//         ros::spinOnce();
//         loop_rate.sleep();

//         // motor2.setLED(0, 0, 255);
//     }

//     // motor2.setLED(0, 0, 0);

//     // motor1.setTorqueEnable(false);
//     // motor2.setTorqueEnable(false);
//     // motor3.setTorqueEnable(false);
//     // motor4.setTorqueEnable(false);
//     // motor5.setTorqueEnable(false);
//     // motor6.setTorqueEnable(false);

//     return 0;
// }