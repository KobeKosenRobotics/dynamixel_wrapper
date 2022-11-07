/**
* @file open_manipulator_control.cpp
* @brief
* @author Akiro Harada
* @date 2022.8.13
* @details
*/
 
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

#define SIMULATION

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

void nextStep();
void updateStep(int next_step);

void setGoalPosition(Eigen::Matrix<double, 6, 1> target_theta);
Eigen::Matrix<double, 6, 1> getPresentPosition();
bool isInPose();
void setTargetPose(double x, double y, double z);
void setTargetPose(double x, double y, double z, double theta3, double theta4, double theta5);

void emagencyStop();
void neutral();

Eigen::Matrix3d rotationX(double theta);
Eigen::Matrix3d rotationY(double theta);
Eigen::Matrix3d rotationZ(double theta);
geometry_msgs::Pose forwardKinematics();
void inverseKinematics();
double getTargetDistance();

void tf_broadcaster(Eigen::Matrix<double, 6, 1> sim_theta);

// Global Variables
// Sequence
namespace Step
{
    bool message = true;
    const int emagency             = 0,
              stay_neutral         = emagency+5,
              stay                 = stay_neutral+5,
              prescan_neutral      = stay+5,
              prescan              = prescan_neutral+5,
              scan                 = prescan+5,
              precatch_neutral     = scan+5,
              precatch             = precatch_neutral+5,
              catcH                = precatch+5,
              prerelease_neutral   = catcH+5,
              prerelease           = prerelease_neutral+5,
              release              = prerelease+5,
              shake1               = release+5,
              shake2               = shake1+5,
              post_release         = shake2+5,
              finish_neutral       = post_release+5,
              finish               = finish_neutral+5;
};

int status = Step::stay_neutral;
int scan_count = 0;
int shaker = 10;
Wait wait, shaker_wait;

#ifdef SIMULATION
Wait sim_wait;
#endif

// Arm Property
Eigen::Matrix<double, 3, 1> link_offset, link0, link1, link2, link3, link4, link5, link_tip;
double l_offset, l0, l1, l2, l3, l4, l5;
Eigen::Matrix<double, 6, 1> homing_offset;

// Enable
bool enable;
bool enable_enable = false;

// Emagency
bool emagency;

// Trash Pose
geometry_msgs::Pose trash_pose;
bool is_scanning = true;
ros::Time start_time;
geometry_msgs::Pose target_pose, start_pose, now_pose;
double val, x, y, z, roll, pitch, yaw, virtual_hight, radius, bowstring, alpha, beta, delta;
bool is_valid = true, is_valid_old = true;
Eigen::Matrix<double, 6, 1> target_theta, theta, global_theta, sim_theta;    // target_theta: to motor, theta: from sensor
#ifndef SIMULATION
double duration_time = 3.0, linear_velocity = 50.0;    // linear_velocity[mm/s]
#endif
#ifdef SIMULATION
double duration_time = 3.0, linear_velocity = 200.0;    // linear_velocity[mm/s]
#endif
bool is_first_inverse_kinematics = true;

// Euler Orientation
Eigen::Matrix<double, 3, 1> sim_euler;
double e0, e1, e2;

// Motor Declaration
#ifndef SIMULATION
std::string port_name("/dev/ttyUSB0");
int baudrate = 1000000;
dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
dynamixel_wrapper::dynamixel_wrapper motor0(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor1(2, dxl_base, dynamixel_wrapper::H54_200_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor2(3, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor3(4, dxl_base, dynamixel_wrapper::H54_100_S500_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor4(5, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
dynamixel_wrapper::dynamixel_wrapper motor5(6, dxl_base, dynamixel_wrapper::H42_020_S300_R, 4);
#endif

// Publisher
std_msgs::Int16 state;

// Callback
void enable_cb(std_msgs::Bool::ConstPtr msg)
{
    enable = msg->data;
}
void emagency_cb(std_msgs::Bool::ConstPtr msg)
{
    if(msg->data == emagency) return;
    emagency = msg->data;
}
void trash_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    if(is_scanning)
    {
        trash_pose.orientation.w = msg->orientation.w;
        if(msg->orientation.w > 0.5)    // exist:w=1, not exsist:w=0;
        {
            trash_pose = *msg;
        }
    }
    
}

// Sequence
void sequence()
{
    if(emagency)
    {
        #ifndef SIMULATION
        motor0.setCurrentLimit(0);
        motor1.setCurrentLimit(1860.0/32.0);
        motor2.setCurrentLimit(930.0/16.0);
        motor3.setCurrentLimit(930.0/16.0);
        motor4.setCurrentLimit(1395.0/32.0);
        motor5.setCurrentLimit(1395.0/32.0);
        #endif
        updateStep(Step::emagency);
    }
    else if(enable_enable && !enable)
    {
        updateStep(Step::stay_neutral);    // Stay Neutral
    }

    // std::cout << "    " << status << "    ";

    switch(status)
    {
        case Step::emagency:    // Emagency
            if(Step::message) std::cout << "EMAGENCY  EMAGENCY  EMAGENCY  EMAGENCY" << std::endl;
            emagencyStop();
            enable = false;
            if(!emagency)
            {
                #ifndef SIMULATION
                motor0.setCurrentLimit(22740.0/16.0);
                motor1.setCurrentLimit(1860.0/8.0);
                motor2.setCurrentLimit(930.0/4.0);
                motor3.setCurrentLimit(930.0/4.0);
                motor4.setCurrentLimit(1395.0/8.0);
                motor5.setCurrentLimit(1395.0/8.0);
                #endif
                updateStep(Step::stay_neutral);
            }
            break;
        
        case Step::stay_neutral:    // Stay Neutral
            enable_enable = false;

            neutral();
            if(isInPose())
            {
                nextStep();
            }
            break;
        
        case Step::stay:    // Stay
            if(Step::message) std::cout << "STAY  STAY  STAY  STAY" << std::endl;
            
            scan_count = 0;

            setTargetPose(0.0, -200.0, 100.0);
            inverseKinematics();
            
            if(enable)
            {
                enable_enable = true;
                nextStep();
            }
            break;

        case Step::prescan_neutral:
            neutral();

            if(isInPose())
            {
                nextStep();
            }
            break;
            
        case Step::prescan:    // PreScan
            if(Step::message) std::cout << "SCAN  SCAN  SCAN  SCAN" << std::endl;

            setTargetPose(448.0, 0.0, 224.0);
            inverseKinematics();

            if(isInPose())
            {
                nextStep();
            }
            break;
        
        case Step::scan:    // Scan
            if(Step::message) std::cout << "SCAN  SCAN  SCAN  SCAN" << std::endl;
            is_scanning = true;
            // if(trash_pose.orientation.w > 0.5 && !wait.isWaiting(3))
            if(trash_pose.orientation.w > 0.5 && !wait.isWaiting(1))
            {
                is_scanning = false;
                nextStep();
            }
            else if(!wait.isWaiting(5))
            {
                updateStep(Step::finish_neutral);
            }
            break;

        case Step::precatch:
            if(Step::message) std::cout << "CATCH  CATCH  CATCH  CATCH" << std::endl;

            setTargetPose(trash_pose.position.x, trash_pose.position.y, trash_pose.position.z+100);
            inverseKinematics();

            // Exit
            if(isInPose())
            {
                nextStep();
            }

            break;
        
        case Step::catcH:    // Catch
            if(Step::message) std::cout << "CATCH  CATCH  CATCH  CATCH" << std::endl;

            setTargetPose(trash_pose.position.x, trash_pose.position.y, trash_pose.position.z);
            inverseKinematics();

            // Exit
            if(isInPose() && !wait.isWaiting(5))
            // if(isInPose() && !wait.isWaiting(2))
            {
                nextStep();
            }

            break;

        case Step::prerelease_neutral:
            neutral();
            if(isInPose())
            {
                nextStep();
            }
            break;
        
        case Step::prerelease:    // PreRelease2
            if(Step::message) std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            // target_theta << -120, -20, 110, -90, 80, 0;
            // target_theta *= M_PI/180.0;
            setTargetPose(200.0, -120.0, 400.0, -1.6, -0.09, 0.0);
            inverseKinematics();

            if(isInPose())
            {
                nextStep();
            }
            break;
        
        case Step::release:    // Release
            if(Step::message) std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            // target_theta << -120, -20, 110, -90, 40, 0;
            // target_theta *= M_PI/180.0;
            setTargetPose(-190.0, -200.0, 400.0, -1.6, -0.09, 0.0);
            inverseKinematics();

            if(isInPose())
            {
                nextStep();
            }
            break;
        
        case Step::shake1:    // Shake1
            if(Step::message) std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            // target_theta << -120, -20, 110, -90+shaker, 40, 0;
            // target_theta *= M_PI/180.0;
            setTargetPose(-192.0, -200.0, 385.0, -1.40, -0.09, 0.0);
            inverseKinematics();

            if(!shaker_wait.isWaiting(5))
            // if(!shaker_wait.isWaiting(2))
            {
                shaker_wait.reset();
                updateStep(Step::post_release);
            }
            if(isInPose())
            {
                updateStep(Step::shake2);
            }
            break;

        case Step::shake2:    // Shake2
            if(Step::message) std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            // target_theta << -120, -20, 110, -90-shaker, 40, 0;
            // target_theta *= M_PI/180.0;
            setTargetPose(-194.0, -202.0, 408.0, -1.75, -0.09, 0.0);
            inverseKinematics();

            if(isInPose())
            {
                updateStep(Step::shake1);
            }
            break;

        case Step::post_release:    // PostRelease
            if(Step::message) std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            setTargetPose(-0.1, -235.0, 405.0, -1.57, 0.0, 0.0);
            inverseKinematics();

            if(scan_count >= 2)
            {
                updateStep(Step::finish_neutral);
            }
            else if(isInPose())
            {
                updateStep(Step::prescan);
            }
            break;

        case Step::finish_neutral:    // Neutral
            enable_enable = false;
            neutral();

            if(isInPose())
            {
                nextStep();
            }
            break;

        case Step::finish:    // Finish
            if(Step::message) std::cout << "FINISH  FINISH  FINISH  FINISH" << std::endl;
            enable_enable = false;

            setTargetPose(0.0, -200.0, 100.0);
            inverseKinematics();

            if(isInPose() && !wait.isWaiting(5))
            {
                updateStep(Step::stay);
            }
            break;
            
        default:
            nextStep();
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
    ros::Subscriber emagency_sub = nh.subscribe<std_msgs::Bool>("emagency", 10, emagency_cb);
    ros::Subscriber trash_sub = nh.subscribe<geometry_msgs::Pose>("trash_pose", 10, trash_pose_cb);
    
    // Arm Property
    link_offset << 0.0, 0.0, 159.0;
    link0 << 0.0, 0.0, 0.0;
    link1 << 30.0, 0.0, 264.0;
    link2 << -30.0, 0.0, 258.0;
    link3 << 0.0, 0.0, 0.0;
    link4 << 0.0, 0.0, 123.0;    // Without End Effector
    link5 << 0.0, 0.0, 0.0;
    link_tip = link3+link4+link5;

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
    #ifndef SIMULATION
    motor0.setTorqueEnable(false);
    motor0.setVelocityLimit(15);
    motor0.setCurrentLimit(22740.0/16.0);    // 710    // 64x,32x,16o
    motor0.setHomingOffset(homing_offset(0,0));
    motor0.setPositionGain(15, 0, 0);
    motor0.setTorqueEnable(true);
    
    motor1.setTorqueEnable(false);
    motor1.setVelocityLimit(15);
    motor1.setCurrentLimit(1860.0/8.0);    // 16x,8o
    motor1.setHomingOffset(36.3+homing_offset(1,0));
    motor1.setPositionGain(15, 0, 0);
    motor1.setTorqueEnable(true);
    
    motor2.setTorqueEnable(false);
    motor2.setVelocityLimit(15);
    motor2.setCurrentLimit(930.0/4.0);    // 8x,4o
    motor2.setHomingOffset(45+homing_offset(2,0));
    motor2.setPositionGain(15, 0, 0);
    motor2.setTorqueEnable(true);
    
    motor3.setTorqueEnable(false);
    motor3.setVelocityLimit(15);
    motor3.setCurrentLimit(930.0/4.0);    // 8x,4o
    motor3.setHomingOffset(homing_offset(3,0));
    motor3.setPositionGain(15, 0, 0);
    motor3.setTorqueEnable(true);
    
    motor4.setTorqueEnable(false);
    motor4.setVelocityLimit(15);
    motor4.setCurrentLimit(1395.0/8.0);    // 16x,8o
    motor4.setHomingOffset(homing_offset(4,0));
    motor4.setPositionGain(15, 0, 0);
    motor4.setTorqueEnable(true);
    
    motor5.setTorqueEnable(false);
    motor5.setVelocityLimit(15);
    motor5.setCurrentLimit(1395.0/8.0);
    motor5.setHomingOffset(homing_offset(5,0));
    motor3.setPositionGain(15, 0, 0);
    motor5.setTorqueEnable(true);
    #endif

    homing_offset *= M_PI/180.0;
    
    // Start Time
    start_time = ros::Time::now();

    target_theta << 0.0, -0.017, 1.788, 0.0, 1.371, 0.0; 

    #ifdef SIMULATION
    sim_theta = target_theta;
    #endif

    while(nh.ok())
    {
        tf_broadcaster(sim_theta);
        now_pose = forwardKinematics();
        
        sequence();
        
        std::cout << val << std::endl;
        // std::cout << now_pose << std::endl;
        // std::cout << target_theta*180.0/M_PI << std::endl;
        
        if(is_valid)
        {
            #ifndef SIMULATION
            setGoalPosition(target_theta);
            #endif
        }

        #ifdef SIMULATION
        sim_theta = target_theta;
        #endif
        
        state.data = status;
        state_pub.publish(state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Torque Off
    #ifndef SIMULATION
    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);
    #endif

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

    _duration_seconds = _duration.toSec();

    // std::cout << _duration_seconds << std::endl;

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

void nextStep()
{
    wait.reset();
    is_first_inverse_kinematics = true;
    status++;
}
void updateStep(int next_step)
{
    wait.reset();
    is_first_inverse_kinematics = true;
    status = next_step;
}

// Collective Instruction
void setGoalPosition(Eigen::Matrix<double, 6, 1> target_theta)
{
    #ifdef SIMULATION
    sim_theta = target_theta;
    #endif
    
    #ifndef SIMULATION
    target_theta *= 180.0/M_PI;
    motor0.setGoalPosition(target_theta(0,0));
    motor1.setGoalPosition(target_theta(1,0));
    motor2.setGoalPosition(target_theta(2,0));
    motor3.setGoalPosition(target_theta(3,0));
    motor4.setGoalPosition(target_theta(4,0));
    motor5.setGoalPosition(target_theta(5,0));
    #endif
}
Eigen::Matrix<double, 6, 1> getPresentPosition()
{
    #ifndef SIMULATION
    Eigen::Matrix<double, 6, 1> present_position;
    present_position(0,0) = motor0.getPresentPosition();
    present_position(1,0) = motor1.getPresentPosition();
    present_position(2,0) = motor2.getPresentPosition();
    present_position(3,0) = motor3.getPresentPosition();
    present_position(4,0) = motor4.getPresentPosition();
    present_position(5,0) = motor5.getPresentPosition();
    present_position *= M_PI/180.0;
    return present_position;
    #endif

    #ifdef SIMULATION
    return sim_theta;
    #endif
}

bool isInPose()
{
    double dt1, dt2, dt3, dt4, dt5, dt6;
    dt1 = fabs(target_pose.position.x - now_pose.position.x);
    dt2 = fabs(target_pose.position.x - now_pose.position.x);
    dt3 = fabs(target_pose.position.x - now_pose.position.x);
    dt4 = fabs(target_pose.orientation.x - now_pose.orientation.x);
    dt5 = fabs(target_pose.orientation.y - now_pose.orientation.y);
    dt6 = fabs(target_pose.orientation.z - now_pose.orientation.z);
    if(dt1 < 1.0 && dt2 < 1.0 && dt3 < 1.0 && dt4 < 0.1 && dt5 < 0.1 && dt6 < 0.1) return true;
    
    return false;
}

void setTargetPose(double x, double y, double z)
{
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
}

void setTargetPose(double x, double y, double z, double theta3rad, double theta4rad, double theta5rad)
{
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = theta3rad;
    target_pose.orientation.y = theta4rad;
    target_pose.orientation.z = theta5rad;
}

void emagencyStop()
{
    #ifndef SIMULATION
    // target_theta = getPresentPosition();
    target_theta(0,0) = motor0.getPresentPosition();
    target_theta(1,0) = motor1.getPresentPosition();
    target_theta(2,0) = motor2.getPresentPosition();
    target_theta(3,0) = motor3.getPresentPosition();
    target_theta(4,0) = motor4.getPresentPosition();
    target_theta(5,0) = motor5.getPresentPosition();
    
    target_theta *= M_PI/180.0;

    #endif

    #ifdef SIMULATION
    sim_theta = target_theta;
    #endif

    return;
}
void neutral()
{
    if(Step::message) std::cout << "NEUTRAL  NEUTRAL  NEUTRAL  NEUTRAL"  << std::endl;
    setTargetPose(250.0, 0.0, 250.0);
    inverseKinematics();
    return;
}

// Basic Rotation
Eigen::Matrix3d rotationX(double global_theta)
{    
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0              ,  0.0              ,
                0.0, cos(global_theta), -sin(global_theta),
                0.0, sin(global_theta),  cos(global_theta);
    return rotation;
}
Eigen::Matrix3d rotationY(double global_theta)
{
    Eigen::Matrix3d rotation;
    rotation <<  cos(global_theta), 0.0, sin(global_theta),
                 0.0              , 1.0, 0.0              ,
                -sin(global_theta), 0.0, cos(global_theta);
    return rotation;
}
Eigen::Matrix3d rotationZ(double global_theta)
{
    Eigen::Matrix3d rotation;
    rotation << cos(global_theta), -sin(global_theta), 0.0,
                sin(global_theta),  cos(global_theta), 0.0,
                0.0              ,  0.0              , 1.0;
    return rotation;
}

// Forward Kinematics
geometry_msgs::Pose forwardKinematics()
{
    // Get Theta
    #ifndef SIMULATION
    theta << motor0.getPresentPosition(),
             motor1.getPresentPosition(),
             motor2.getPresentPosition(),
             motor3.getPresentPosition(),
             motor4.getPresentPosition(),
             motor5.getPresentPosition();
    theta *= M_PI/180.0;
    #endif
    
    #ifdef SIMULATION
    theta = sim_theta;
    #endif

    global_theta = theta-homing_offset;

    // Rotation
    Eigen::Matrix<double, 3, 3> rotation[6];
    rotation[0] = rotationZ(global_theta(0,0));
    rotation[1] = rotationY(global_theta(1,0));
    rotation[2] = rotationY(global_theta(2,0));
    rotation[3] = rotationZ(global_theta(3,0));
    rotation[4] = rotationY(global_theta(4,0));
    rotation[5] = rotationZ(global_theta(5,0));

    // Position
    Eigen::Matrix<double, 3, 1> position;
    position = link_offset+rotation[0]*(link0+rotation[1]*(link1+rotation[2]*(link2+rotation[3]*(link3+rotation[4]*(link4+rotation[5]*link5)))));

    // Matrix To Pose
    geometry_msgs::Pose pose;
    pose.position.x = position(0,0);
    pose.position.y = position(1,0);
    pose.position.z = position(2,0);
    pose.orientation.x = theta(3,0);
    pose.orientation.y = theta(1,0)+theta(2,0)+theta(4,0)-M_PI;
    pose.orientation.z = theta(5,0);

    return pose;
}

void inverseKinematics()
{
    // Start
    if(is_first_inverse_kinematics)
    {
        start_pose = forwardKinematics();
        duration_time = getTargetDistance()/linear_velocity;
        start_time = ros::Time::now();
        is_first_inverse_kinematics = false;
    }

    // Get Theta
    #ifndef SIMULATION
    theta << motor0.getPresentPosition(),
             motor1.getPresentPosition(),
             motor2.getPresentPosition(),
             motor3.getPresentPosition(),
             motor4.getPresentPosition(),
             motor5.getPresentPosition();
    theta *= M_PI/180.0;
    #endif
    
    #ifdef SIMULATION
    theta = sim_theta;
    #endif

    global_theta = theta-homing_offset;
    
    // Liner Interpolation
    val = std::min(std::max((ros::Time::now() - start_time).toSec()/duration_time, 0.0), 1.0);
    x = start_pose.position.x*(1.0-val) + target_pose.position.x*val;
    y = start_pose.position.y*(1.0-val) + target_pose.position.y*val;
    z = start_pose.position.z*(1.0-val) + target_pose.position.z*val;
    roll  = start_pose.orientation.x*(1.0-val) + target_pose.orientation.x*val;
    pitch = start_pose.orientation.y*(1.0-val) + target_pose.orientation.y*val;
    yaw   = start_pose.orientation.z*(1.0-val) + target_pose.orientation.z*val;

    // Tip
    // Rotation
    Eigen::Matrix<double, 3, 3> rotation[6];
    rotation[0] = rotationZ(global_theta(0,0));
    rotation[1] = rotationY(global_theta(1,0));
    rotation[2] = rotationY(global_theta(2,0));
    rotation[3] = rotationZ(global_theta(3,0));
    rotation[4] = rotationY(global_theta(4,0));
    rotation[5] = rotationZ(global_theta(5,0));

    Eigen::Matrix<double, 3, 1> now_link_tip;

    now_link_tip = rotation[0]*(rotation[1]*(rotation[2]*(rotation[3]*(link3+rotation[4]*(link4+rotation[5]*link5)))));

    x -= now_link_tip(0,0);
    y -= now_link_tip(1,0);
    z -= now_link_tip(2,0);

    // Intermediate Calculation
    virtual_hight = z-l_offset;
    radius = sqrt(x*x+y*y);
    bowstring = sqrt(virtual_hight*virtual_hight+radius*radius);
    alpha = acos((bowstring*bowstring+l1*l1-l2*l2)/(2*bowstring*l1));
    beta = acos((l1*l1+l2*l2-bowstring*bowstring)/(2*l1*l2));
    delta = acos(virtual_hight/bowstring);
    if(radius/bowstring < 0)
    {
        delta *= (-1);
    }

    // Target Theta Position
    target_theta(0,0) = acos(x/radius);
    if(y/radius < 0)
    {
        target_theta(0,0) *= (-1);
    }
    target_theta(1,0) = delta-alpha;
    target_theta(2,0) = M_PI-beta;

    // Target Theta Orientation
    target_theta(3,0) = roll;
    target_theta(4,0) = M_PI-target_theta(1,0)-target_theta(2,0)+pitch;
    target_theta(5,0) = yaw;
    
    // Check
    for(int i = 0; i < 6; i++)
    {
        if(isnan(target_theta(i,0)))
        {
            is_valid = false;
            false;
        }
    }

    // LED
    if(is_valid != is_valid_old)
    {
        #ifndef SIMULATION
        motor0.setLED(is_valid?0:255, is_valid?255:0, 0);
        motor1.setLED(is_valid?0:255, is_valid?255:0, 0);
        motor2.setLED(is_valid?0:255, is_valid?255:0, 0);
        motor3.setLED(is_valid?0:255, is_valid?255:0, 0);
        motor4.setLED(is_valid?0:255, is_valid?255:0, 0);
        motor5.setLED(is_valid?0:255, is_valid?255:0, 0);
        #endif
    }
    is_valid_old = is_valid;
}

double getTargetDistance()
{
    return sqrt(pow(target_pose.position.x-start_pose.position.x,2)+pow(target_pose.position.y-start_pose.position.y,2)+pow(target_pose.position.z-start_pose.position.z,2));
}

void tf_broadcaster(Eigen::Matrix<double, 6, 1> sim_theta)
{
    Eigen::Matrix<double, 6, 1> sim_global_theta = sim_theta - homing_offset;

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
    
    q.setRPY(0, 0, sim_global_theta(0,0));
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
    
    q.setRPY(0, sim_global_theta(1,0), 0);
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
    
    q.setRPY(0, sim_global_theta(2,0), 0);
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
    
    q.setRPY(0, 0, sim_global_theta(3,0));
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
    
    q.setRPY(0, sim_global_theta(4,0), 0);
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
    
    q.setRPY(0, 0, sim_global_theta(5,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}