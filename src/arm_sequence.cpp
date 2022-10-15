/**
* @file open_manipulator_control.cpp
* @brief
* @author Akiro Harada
* @date 2022.8.13
* @details
*/
 
#include <ros/ros.h>
 
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

// Prototype Declearation
class Wait
{
    private:
        ros::Time _start, _end;
        bool _is_first = true;
        ros::Duration _duration;
        double _duration_seconds;
    
    public:
        bool isWaiting(double seconds);
        void reset();
};


void setGoalPosition(Eigen::Matrix<double, 6, 1> target_theta);
bool isInPosition(double t1, double t2, double t3, double t4, double t5, double t6);
bool isInPosition(Eigen::Matrix<double, 6, 1> target_theta);

Eigen::Matrix3d rotationX(double theta);
Eigen::Matrix3d rotationY(double theta);
Eigen::Matrix3d rotationZ(double theta);
geometry_msgs::Pose forwardKinematics();

// Global Variables
// Sequence
int status = 10;
int scan_count = 0;
int shaker = 10;
Wait wait, getIsInPos;    // getIsInPose->wait??

// Arm Property
Eigen::Matrix<double, 3, 1> link0, link1, link2, link3, link4, link5;
double l1 = 0.0, l2 = sqrt(pow(264.0,2.0)+pow(30.0,2.0)), l3 = sqrt(pow(258.0,2.0)+pow(30.0,2.0)), l4 = 0.0, l5 = 123.0, l6 = 0.0;

// Enable
bool enable;

// Trash Pose
geometry_msgs::Pose trash_pose;
bool is_scanning = true;
ros::Time start_time;
geometry_msgs::Pose target_pose,start_pose, now_pose;
double val, x, y, z, virtual_hight, radius, bowstring, alpha, beta, delta;
bool is_valid = true, is_valid_old = true;
Eigen::Matrix<double, 6, 1> target_theta;    // target_theta: to motor, theta: from sensor
double duration_time = 3.0;

// Motor Declaration
std::string port_name("/dev/ttyUSB0");
int baudrate=1000000;
dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor2(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor3(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor4(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor5(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor6(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);

// Publisher
std_msgs::Int16 state;

// Callback
void enable_cb(std_msgs::Bool::ConstPtr msg)
{
    if(msg->data == enable) return;
    enable = msg->data;
    if(enable)
    {
        status++;
    }
    else
    {
        status = 0;
    }
}
void trash_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    if(is_scanning && msg->orientation.w > 0)    // exist:w=1, not exsist:w=-1;
    {
        if(*msg == target_pose) return;
        trash_pose = *msg;
        start_pose = forwardKinematics();
        if(msg->orientation.x > 3.0)
        {
            duration_time = msg->orientation.x;
        }
        start_time = ros::Time::now();
    }
}

// Sequence
void sequence()
{
    switch(status)
    {
        case 0:    // Stay
            std::cout << "STAY  STAY  STAY  STAY" << std::endl;
            enable = false;
            // target_theta << -90, 0, 140, 0, 40, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-90);
            motor2.setGoalPosition(0);
            motor3.setGoalPosition(140);
            motor4.setGoalPosition(0);
            motor5.setGoalPosition(40);
            motor6.setGoalPosition(0);
            break;

        case 2:    // PreScan
            std::cout << "PRESCAN  PRESCAN  PRESCAN  PRESCAN" << std::endl;
            // target_theta << 0, 45, 45, 0, 90, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(0);
            motor2.setGoalPosition(45);
            motor3.setGoalPosition(45);
            motor4.setGoalPosition(0);
            motor5.setGoalPosition(90);
            motor6.setGoalPosition(0);
            if(isInPosition(0,45,45,0,90,0))
            {
                status++;
            }
            break;
        
        case 5:    // Scan
            std::cout << "SCAN  SCAN  SCAN  SCAN" << std::endl;
            is_scanning = true;
            if(trash_pose.orientation.w > 0 && !wait.isWaiting(10))
            {
                is_scanning = false;
                scan_count++;
                wait.reset();
                status++;
            }
            else if(!wait.isWaiting(10))
            {
                status = 30;
            }
            break;
        
        case 10:    // Catch
            std::cout << "CATCH  CATCH  CATCH  CATCH" << std::endl;
            
            // Liner Interpolation
            val = std::min(std::max((ros::Time::now() - start_time).toSec()/duration_time, 0.0), 1.0);
            x = now_pose.position.x = start_pose.position.x*(1.0-val) + target_pose.position.x*val;
            y = now_pose.position.y = start_pose.position.y*(1.0-val) + target_pose.position.y*val;
            z = now_pose.position.z = start_pose.position.z*(1.0-val) + target_pose.position.z*val;

            // Intermediate Calculation
            virtual_hight = z+l5;
            radius = sqrt(x*x+y*y);
            bowstring = sqrt(virtual_hight*virtual_hight+radius*radius);
            alpha = acos((bowstring*bowstring+l2*l2-l3*l3)/(2*bowstring*l2));
            beta = acos((l2*l2+l3*l3-bowstring*bowstring)/(2*l2*l3));
            if(virtual_hight>0)
            {
                delta = atan(radius/virtual_hight);
            }
            else if(virtual_hight)
            {
                delta = M_PI+atan(radius/virtual_hight);
            }

            // Target Theta
            if(x>0)
            {
                target_theta(0,0) = atan(y/x);
            }
            else if(x<0)
            {
                target_theta(0,0) = -atan(y/x);
            }
            target_theta(1,0) = delta-alpha;
            target_theta(2,0) = M_PI-beta;
            target_theta(3,0) = 0;
            target_theta(4,0) = M_PI-target_theta(1,0)-target_theta(2,0);
            target_theta(5,0) = 0;

            target_theta = target_theta*180.0/M_PI;

            std::cout << target_theta << std::endl;
            
            // Check
            for(int i = 0; i < 6; i++)
            {
                if(isnan(target_theta(i,0)))
                {
                    is_valid = false;
                    false;
                }
            }

            // Output
            if(is_valid)
            {
                setGoalPosition(target_theta);
            }
            if(is_valid != is_valid_old)
            {
                motor1.setLED(is_valid?0:255, is_valid?255:0, 0);
                motor2.setLED(is_valid?0:255, is_valid?255:0, 0);
                motor3.setLED(is_valid?0:255, is_valid?255:0, 0);
                motor4.setLED(is_valid?0:255, is_valid?255:0, 0);
                motor5.setLED(is_valid?0:255, is_valid?255:0, 0);
                motor6.setLED(is_valid?0:255, is_valid?255:0, 0);
            }
            is_valid_old = is_valid;

            // Exit
            // if(wait.isWaiting(5))
            // {
            //     trash_pose.orientation.w = -1;
            //     trash_pose.position.x = 0.0;
            //     trash_pose.position.y = 0.0;
            //     trash_pose.position.z = 0.0;
                
            //     status++;
            // }

            break;
        
        case 15:    // PreRelease1
            std::cout << "PRERELEASE1  PRERELEASE1  PRERELEASE1  PRERELEASE1" << std::endl;
            // target_theta << 0, 45, 45, 0, 90, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(0);
            motor2.setGoalPosition(45);
            motor3.setGoalPosition(45);
            motor4.setGoalPosition(0);
            motor5.setGoalPosition(90);
            motor6.setGoalPosition(0);
            if(isInPosition(0,45,45,0,90,0) && !wait.isWaiting(1))
            {
                status++;
            }
            break;
        
        case 17:    // PreRelease2
            std::cout << "PRERELEASE2  PRERELEASE2  PRERELEASE2  PRERELEASE2" << std::endl;
            // target_theta << -120, -20, 110, -90, 80, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-120);
            motor2.setGoalPosition(-20);
            motor3.setGoalPosition(110);
            motor4.setGoalPosition(-90);
            motor5.setGoalPosition(80);
            motor6.setGoalPosition(0);
            if(isInPosition(-90,-20,110,-90,80,0))
            {
                status++;
            }
            break;
        
        case 20:    // Release
            std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            // target_theta << -120, -20, 110, -90, 40, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-120);
            motor2.setGoalPosition(-20);
            motor3.setGoalPosition(110);
            motor4.setGoalPosition(-90);
            motor5.setGoalPosition(40);
            motor6.setGoalPosition(0);
            if(isInPosition(-120,-20,110,-90,40,0))
            {
                status++;
            }
            break;
        
        case 22:    // Shake
            std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            // target_theta << -120, -20, 110, -90+shaker, 40, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-120);
            motor2.setGoalPosition(-20);
            motor3.setGoalPosition(110);
            motor4.setGoalPosition(-90+shaker);
            motor5.setGoalPosition(40);
            motor6.setGoalPosition(0);
            if(!wait.isWaiting(5))
            {
                status+=2;
            }
            if(isInPosition(-120,-20,110,-90+shaker,40,0))
            {
                status++;
            }
            break;

        case 23:    // Shake
            std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            // target_theta << -120, -20, 110, -90-shaker, 40, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-120);
            motor2.setGoalPosition(-20);
            motor3.setGoalPosition(110);
            motor4.setGoalPosition(-90-shaker);
            motor5.setGoalPosition(40);
            motor6.setGoalPosition(0);
            if(isInPosition(-120,-20,110,-90-shaker,40,0))
            {
                status--;
            }
            break;

        case 25:    // PostRelease
            std::cout << "POSTRELEASE  POSTRELEASE  POSTRELEASE  POSTRELEASE" << std::endl;
            // target_theta << -60, -20, 110, -90, 80, 0;
            // setGoalPosition(target_theta);
            motor1.setGoalPosition(-60);
            motor2.setGoalPosition(-20);
            motor3.setGoalPosition(110);
            motor4.setGoalPosition(-90);
            motor5.setGoalPosition(80);
            motor6.setGoalPosition(0);
            if(scan_count > 5)
            {
                status = 2;
            }
            else if(isInPosition(-60,-20,110,-90,80,0) && !wait.isWaiting(1))
            {
                status = 2;
            }
            break;

        case 30:    // Finish
            std::cout << "FINISH  FINISH  FINISH  FINISH" << std::endl;
            target_theta << -90, 0, 140, 0, 40, 0;
            setGoalPosition(target_theta);
            if(!wait.isWaiting(5))
            {
                status = 0;
            }
            break;
            
        default:
            wait.reset();
            status++;
            break;
    }
}
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArmSequence");
    ros::NodeHandle nh;
    double rate = 10.0f;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher state_pub = nh.advertise<std_msgs::Int16>("state", 1);

    // Subscriber
    ros::Subscriber enable_sub = nh.subscribe<std_msgs::Bool>("enable", 10, enable_cb);
    ros::Subscriber trash_sub = nh.subscribe<geometry_msgs::Pose>("trash_pose", 10, trash_pose_cb);
    
    // Link Matrix
    link0 << 0.0, 0.0, l1;
    link1 << 0.0, 0.0, l2;
    link2 << 0.0, 0.0, l3;
    link3 << 0.0, 0.0, l4;
    link4 << 0.0, 0.0, l5;    // Without End Effector
    link5 << 0.0, 0.0, l6;

    // Motor Set Up
    motor1.setTorqueEnable(false);
    motor1.setVelocityLimit(15);
    motor1.setHomingOffset(0);
    motor1.setPositionGain(15, 0, 0);
    motor1.setTorqueEnable(true);
    
    motor2.setTorqueEnable(false);
    motor2.setVelocityLimit(15);
    motor2.setHomingOffset(36.3);
    motor2.setPositionGain(15, 0, 0);
    motor2.setTorqueEnable(true);
    
    motor3.setTorqueEnable(false);
    motor3.setVelocityLimit(15);
    motor3.setHomingOffset(45);
    motor3.setPositionGain(15, 0, 0);
    motor3.setTorqueEnable(true);
    
    motor4.setTorqueEnable(false);
    motor4.setVelocityLimit(15);
    motor4.setHomingOffset(0);
    motor4.setPositionGain(15, 0, 0);
    motor4.setTorqueEnable(true);
    
    motor5.setTorqueEnable(false);
    motor5.setVelocityLimit(15);
    motor5.setHomingOffset(0);
    motor5.setPositionGain(15, 0, 0);
    motor5.setTorqueEnable(true);
    
    motor6.setTorqueEnable(false);
    motor6.setVelocityLimit(15);
    motor6.setHomingOffset(0);
    motor4.setPositionGain(15, 0, 0);
    motor6.setTorqueEnable(true);
    
    // Start Time
    start_time = ros::Time::now();

    while(nh.ok())
    {
        sequence();
        
        state.data = status;
        state_pub.publish(state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Torque Off
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);
    motor6.setTorqueEnable(false);

    return 0;
}

// Wait
bool Wait::isWaiting(double seconds)
{
    if(_is_first)
    {
        _start = ros::Time::now();
        _is_first = false;
    }

    _end = ros::Time::now();

    _duration = _end-_start;

    _duration_seconds = _duration.toSec();;

    std::cout << _duration_seconds << std::endl;

    if(_duration_seconds < seconds)
    {
        return true;
    }

    _is_first = true;
    return false;
}
void Wait::reset()
{
    _is_first = true;
}

// Collective Instruction
void setGoalPosition(Eigen::Matrix<double, 6, 1> target_theta)
{
    motor1.setGoalPosition(target_theta(0,0));
    motor2.setGoalPosition(target_theta(1,0));
    motor3.setGoalPosition(target_theta(2,0));
    motor4.setGoalPosition(target_theta(3,0));
    motor5.setGoalPosition(target_theta(4,0));
    motor6.setGoalPosition(target_theta(5,0));
}
bool isInPosition(double t1, double t2, double t3, double t4, double t5, double t6)
{
    double dt1, dt2, dt3, dt4, dt5, dt6;
    dt1 = fabs(t1 - motor1.getPresentPosition());
    dt2 = fabs(t2 - motor2.getPresentPosition());
    dt3 = fabs(t3 - motor3.getPresentPosition());
    dt4 = fabs(t4 - motor4.getPresentPosition());
    dt5 = fabs(t5 - motor5.getPresentPosition());
    dt6 = fabs(t6 - motor6.getPresentPosition());
    if(dt2 < 1 && dt2 < 1 && dt3 < 1 && dt4 < 1 && dt5 < 1 && dt6 < 1) return true;
    return false;
}
bool isInPosition(Eigen::Matrix<double, 6, 1> target_theta)
{
    double dt1, dt2, dt3, dt4, dt5, dt6;
    dt1 = fabs(target_theta(0,0)- motor1.getPresentPosition());
    dt2 = fabs(target_theta(1,0) - motor2.getPresentPosition());
    dt3 = fabs(target_theta(2,0) - motor3.getPresentPosition());
    dt4 = fabs(target_theta(3,0) - motor4.getPresentPosition());
    dt5 = fabs(target_theta(4,0) - motor5.getPresentPosition());
    dt6 = fabs(target_theta(5,0) - motor6.getPresentPosition());
    if(dt2 < 1 && dt2 < 1 && dt3 < 1 && dt4 < 1 && dt5 < 1 && dt6 < 1) return true;
    return false;
}

// Basic Rotation
Eigen::Matrix3d rotationX(double theta)
{
   Eigen::Matrix3d rotation;
   rotation << 1.0, 0.0       ,  0.0       ,
               0.0, cos(theta), -sin(theta),
               0.0, sin(theta),  cos(theta);
   return rotation;
}
Eigen::Matrix3d rotationY(double theta)
{
   Eigen::Matrix3d rotation;
   rotation <<  cos(theta), 0.0, sin(theta),
                0.0       , 1.0, 0.0,
               -sin(theta), 0.0, cos(theta);
   return rotation;
}
Eigen::Matrix3d rotationZ(double theta)
{
   Eigen::Matrix3d rotation;
   rotation << cos(theta), -sin(theta), 0.0,
               sin(theta),  cos(theta), 0.0,
               0.0       , 0.0        , 1.0;
   return rotation;
}

// Forward Kinematics
geometry_msgs::Pose forwardKinematics()
{
    // Get Theta
    Eigen::Matrix<double, 6, 1> theta;
    theta << motor1.getPresentPosition(),
             motor2.getPresentPosition(),
             motor3.getPresentPosition(),
             motor4.getPresentPosition(),
             motor5.getPresentPosition(),
             motor6.getPresentPosition();
    
    // Rotation
    Eigen::Matrix3d rotation[6];
    rotation[0] = rotationZ(theta(0,0));
    rotation[1] = rotationY(theta(1,0));
    rotation[2] = rotationY(theta(2,0));
    rotation[3] = rotationZ(theta(3,0));
    rotation[4] = rotationY(theta(4,0));
    rotation[5] = rotationZ(theta(5,0));

    // Position
    Eigen::Matrix<double, 3, 1> position;
    position = rotation[0]*(link0+rotation[1]*(link1+rotation[2]*(link2+rotation[3]*(link3+rotation[4]*(link4+rotation[5]*link5)))));

    // Matrix To Pose
    geometry_msgs::Pose pose;
    pose.position.x = position(0,0);
    pose.position.y = position(1,0);
    pose.position.z = position(2,0);

    return pose;
}