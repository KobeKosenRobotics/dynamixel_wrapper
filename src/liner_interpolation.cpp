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
bool isInPosition(Eigen::Matrix<double, 6, 1> target_theta);
void setTargetPose(double x, double y, double z);

void emagencyStop();
void neutral();

Eigen::Matrix3d rotationX(double theta);
Eigen::Matrix3d rotationY(double theta);
Eigen::Matrix3d rotationZ(double theta);
geometry_msgs::Pose forwardKinematics();
void inverseKinematics();

void tf_broadcaster(Eigen::Matrix<double, 6, 1> theta);

// Global Variables
// Sequence
namespace Step
{
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
Eigen::Matrix<double, 3, 1> link_offset, link0, link1, link2, link3, link4, link5;
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
double val, x, y, z, virtual_hight, radius, bowstring, alpha, beta, delta;
bool is_valid = true, is_valid_old = true;
Eigen::Matrix<double, 6, 1> target_theta;    // target_theta: to motor, theta: from sensor
double duration_time = 3.0;
bool is_first_inverse_kinematics = true;

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
            if(msg->orientation.x > 3.0)
            {
                duration_time = msg->orientation.x;
            }
        }
    }
    
}

// Sequence
void sequence()
{
    if(emagency)
    {
        updateStep(Step::emagency);
    }
    else if(enable_enable && !enable)
    {
        updateStep(Step::stay_neutral);    // Stay Neutral
    }

    std::cout << status << std::endl;

    switch(status)
    {
        case Step::emagency:    // Emagency
            std::cout << "EMAGENCY  EMAGENCY  EMAGENCY  EMAGENCY" << std::endl;
            emagencyStop();
            enable = false;
            if(!emagency)
            {
                updateStep(Step::stay_neutral);
            }
            break;
        
        case Step::stay_neutral:    // Stay Neutral
            enable_enable = false;
            neutral();
            if(!wait.isWaiting(3))
            {
                nextStep();
            }
            break;
        
        case Step::stay:    // Stay
            std::cout << "STAY  STAY  STAY  STAY" << std::endl;
            
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

            if(isInPosition(target_theta) && !wait.isWaiting(3))
            {
                nextStep();
            }
            break;
            

        case Step::prescan:    // PreScan
            std::cout << "SCAN  SCAN  SCAN  SCAN" << std::endl;

            setTargetPose(448.0, 0.0, 224.0);
            inverseKinematics();

            if(isInPosition(target_theta) && !wait.isWaiting(3))
            {
                nextStep();
            }
            break;
        
        case Step::scan:    // Scan
            std::cout << "SCAN  SCAN  SCAN  SCAN" << std::endl;
            is_scanning = true;
            std::cout << trash_pose.orientation.w << std::endl;
            if(trash_pose.orientation.w > 0.5 && !wait.isWaiting(3))
            {
                is_scanning = false;
                nextStep();
            }
            else if(!wait.isWaiting(5))
            {
                status = 30;
                updateStep(Step::finish_neutral);
            }
            break;

        case Step::precatch:
            std::cout << "CATCH  CATCH  CATCH  CATCH" << std::endl;

            setTargetPose(trash_pose.position.x, trash_pose.position.y, trash_pose.position.z+50);
            inverseKinematics();

            // Exit
            if(!wait.isWaiting(5))
            {
                nextStep();
            }

            break;
        
        case Step::catcH:    // Catch
            std::cout << "CATCH  CATCH  CATCH  CATCH" << std::endl;

            target_pose = trash_pose;
            inverseKinematics();

            // Exit
            if(!wait.isWaiting(5))
            {
                nextStep();
            }

            break;

        case Step::prerelease_neutral:
            neutral();
            if(!wait.isWaiting(3))
            {
                nextStep();
            }
            break;
        
        case Step::prerelease:    // PreRelease2
            std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            target_theta << -120, -20, 110, -90, 80, 0;
            target_theta *= M_PI/180.0;

            if(isInPosition(target_theta))
            {
                nextStep();
            }
            break;
        
        case Step::release:    // Release
            std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            target_theta << -120, -20, 110, -90, 40, 0;
            target_theta *= M_PI/180.0;

            if(isInPosition(target_theta))
            {
                nextStep();
            }
            break;
        
        case Step::shake1:    // Shake1
            std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            target_theta << -120, -20, 110, -90+shaker, 40, 0;
            target_theta *= M_PI/180.0;

            if(!shaker_wait.isWaiting(5))
            {
                shaker_wait.reset();
                updateStep(Step::post_release);
            }
            if(isInPosition(target_theta))
            {
                updateStep(Step::shake2);
            }
            break;

        case Step::shake2:    // Shake2
            std::cout << "SHAKE  SHAKE  SHAKE  SHAKE" << std::endl;
            target_theta << -120, -20, 110, -90-shaker, 40, 0;
            target_theta *= M_PI/180.0;

            if(isInPosition(target_theta))
            {
                updateStep(Step::shake1);
            }
            break;

        case Step::post_release:    // PostRelease
            std::cout << "RELEASE  RELEASE  RELEASE  RELEASE" << std::endl;
            target_theta << -60, -20, 110, -90, 80, 0;
            target_theta *= M_PI/180.0;

            if(scan_count >= 2)
            {
                updateStep(Step::finish_neutral);
            }
            else if(isInPosition(target_theta) && !wait.isWaiting(1))
            {
                updateStep(Step::prescan);
            }
            break;

        case Step::finish_neutral:    // Neutral
            enable_enable = false;
            neutral();

            if(isInPosition(target_theta) && !wait.isWaiting(3))
            {
                nextStep();
            }
            break;

        case Step::finish:    // Finish
            std::cout << "FINISH  FINISH  FINISH  FINISH" << std::endl;
            enable_enable = false;

            setTargetPose(0.0, -200.0, 100.0);
            inverseKinematics();

            if(!wait.isWaiting(5))
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
    #endif

    homing_offset *= M_PI/180.0;
    
    // Start Time
    start_time = ros::Time::now();

    while(nh.ok())
    {
        sequence();
        now_pose = forwardKinematics();
        std::cout << now_pose << std::endl;
        
        if(is_valid)
        {
            #ifndef SIMULATION
            setGoalPosition(target_theta);
            #endif
        }
        tf_broadcaster(target_theta);
        
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
    return present_position;
    #endif

    #ifdef SIMULATION
    return target_theta;
    #endif
}

bool isInPosition(Eigen::Matrix<double, 6, 1> target_theta)
{
    target_theta *= 180.0/M_PI;

    #ifdef SIMULATION
    if(!sim_wait.isWaiting(1))
    {
        sim_wait.reset();
        return true;
    }
    #endif

    #ifndef SIMULATION
    double dt1, dt2, dt3, dt4, dt5, dt6;
    dt1 = fabs(target_theta(0,0) - motor0.getPresentPosition());
    dt2 = fabs(target_theta(1,0) - motor1.getPresentPosition());
    dt3 = fabs(target_theta(2,0) - motor2.getPresentPosition());
    dt4 = fabs(target_theta(3,0) - motor3.getPresentPosition());
    dt5 = fabs(target_theta(4,0) - motor4.getPresentPosition());
    dt6 = fabs(target_theta(5,0) - motor5.getPresentPosition());
    if(dt2 < 1 && dt2 < 1 && dt3 < 1 && dt4 < 1 && dt5 < 1 && dt6 < 1) return true;
    #endif
    
    return false;
}

void setTargetPose(double x, double y, double z)
{
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
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
    target_theta = target_theta;
    #endif

    return;
}
void neutral()
{
    std::cout << "NEUTRAL  NEUTRAL  NEUTRAL  NEUTRAL"  << std::endl;
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
                    0.0           , 1.0, 0.0              ,
                -sin(global_theta), 0.0, cos(global_theta);
    return rotation;
}
Eigen::Matrix3d rotationZ(double global_theta)
{
    Eigen::Matrix3d rotation;
    rotation << cos(global_theta), -sin(global_theta), 0.0,
                sin(global_theta),  cos(global_theta), 0.0,
                0.0              , 0.0               , 1.0;
    return rotation;
}

// Forward Kinematics
geometry_msgs::Pose forwardKinematics()
{
    // Get Theta
    Eigen::Matrix<double, 6, 1> theta, global_theta;

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
    theta = target_theta;
    #endif

    global_theta = theta-homing_offset;

    // Rotation
    Eigen::Matrix3d rotation[6];
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

    return pose;
}

void inverseKinematics()
{
    // Start
    if(is_first_inverse_kinematics)
    {
        start_pose = forwardKinematics();
        start_time = ros::Time::now();
        is_first_inverse_kinematics = false;
    }
    
    // Liner Interpolation
    val = std::min(std::max((ros::Time::now() - start_time).toSec()/duration_time, 0.0), 1.0);
    x = start_pose.position.x*(1.0-val) + target_pose.position.x*val;
    y = start_pose.position.y*(1.0-val) + target_pose.position.y*val;
    z = start_pose.position.z*(1.0-val) + target_pose.position.z*val;

    // Intermediate Calculation
    virtual_hight = z+l4-l_offset;
    radius = sqrt(x*x+y*y);
    bowstring = sqrt(virtual_hight*virtual_hight+radius*radius);
    alpha = acos((bowstring*bowstring+l1*l1-l2*l2)/(2*bowstring*l1));
    beta = acos((l1*l1+l2*l2-bowstring*bowstring)/(2*l1*l2));
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

void tf_broadcaster(Eigen::Matrix<double, 6, 1> theta)
{
    Eigen::Matrix<double, 6, 1> global_theta = theta-homing_offset;

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
    
    q.setRPY(0, 0, global_theta(0,0));
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
    
    q.setRPY(0, global_theta(1,0), 0);
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
    
    q.setRPY(0, global_theta(2,0), 0);
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
    
    q.setRPY(0, 0, global_theta(3,0));
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
    
    q.setRPY(0, global_theta(4,0), 0);
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
    
    q.setRPY(0, 0, global_theta(5,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}