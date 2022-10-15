#include <ros/ros.h>

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

// Constant
const int Coordinate_DOF = 6, Joint_DOF = 6, Limited_DOF = 3;

// Global Variables
geometry_msgs::Pose pose_old;    // Subscribe

Eigen::Matrix<double, 3, 3> rotation[6], rotation_all, alternating_euler;
Eigen::Matrix<double, Joint_DOF, 1> theta, theta_target, theta_dot;
Eigen::Matrix<double, 3, 1> link0, link1, link2, link3, link4, link5, now_position, now_euler;
Eigen::Matrix<double, Coordinate_DOF, 1> now_position_euler, target_position_euler, initial_position_euler;
Eigen::Matrix<double, 3, Joint_DOF> translation_jacobian, alternating_rotation, rotation_jacobian;
Eigen::Matrix<double, Coordinate_DOF, Joint_DOF> jacobian;
Eigen::Matrix<double, Joint_DOF, Coordinate_DOF> jacobian_inverse; 


double t1, t2, t3, t4, t5, t6;
double td1, td2, td3, td4, td5, td6;
double e1, e2, e3;
double l1, l2, l3, l4, l5, l6;
double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6;

double kp = 1.0;

bool is_valid = true, is_valid_old = true;

// Prototype Declaration
Eigen::Matrix3d rotationX(double theta);
Eigen::Matrix3d rotationY(double theta);
Eigen::Matrix3d rotationZ(double theta);
bool is_in_target();
// Eigen::Matrix<double, Joint_DOF, Limited_DOF> getGeneralezedInverse(Eigen::Matrix<double, Limited_DOF, Joint_DOF> M);    // Orientation


ros::Publisher state_pub;
void pose_cb(geometry_msgs::Pose::ConstPtr pose)
{
    if(*pose == pose_old) return;
    target_position_euler(0,0) = pose->position.x;
    target_position_euler(1,0) = pose->position.y;
    target_position_euler(2,0) = pose->position.z;
    target_position_euler(3,0) = pose->orientation.x;
    target_position_euler(4,0) = pose->orientation.y;
    target_position_euler(5,0) = pose->orientation.z;
    pose_old = *pose;
    std_msgs::Bool msg;
    msg.data = false;
    state_pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "dynamixel_wrapper_example_node");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 1, pose_cb);

    state_pub = nh.advertise<std_msgs::Bool>("open_manipulator_controller/isGoaled", 1);

    double rate = 10.0f;
    ros::Rate loop_rate(rate);

    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;

    // Motor
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor2(2, dxl_base, dynamixel_wrapper::PH54_200_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor3(3, dxl_base, dynamixel_wrapper::PH54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor4(4, dxl_base, dynamixel_wrapper::PH54_100_S500_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor5(5, dxl_base, dynamixel_wrapper::PH42_020_S300_R, 1);
    dynamixel_wrapper::dynamixel_wrapper motor6(6, dxl_base, dynamixel_wrapper::PH42_020_S300_R, 1);

    motor1.setTorqueEnable(false);
    motor1.setVelocityLimit(15);
    motor1.setHomingOffset(0);
    motor1.setPositionGain(15, 0, 0);
    motor1.setVelocityGain(1, 0);
    motor1.setTorqueEnable(true);

    motor2.setTorqueEnable(false);
    motor2.setVelocityLimit(15);
    motor2.setHomingOffset(6.383073693);
    motor2.setPositionGain(15, 0, 0);
    motor2.setVelocityGain(1, 0);
    motor2.setTorqueEnable(true);

    motor3.setTorqueEnable(false);
    motor3.setVelocityLimit(15);
    motor3.setHomingOffset(45-6.383073693-6.632514615);
    motor3.setPositionGain(15, 0, 0);
    motor3.setVelocityGain(1, 0);
    motor3.setTorqueEnable(true);

    motor4.setTorqueEnable(false);
    motor4.setVelocityLimit(15);
    motor4.setHomingOffset(0);
    motor4.setPositionGain(15, 0, 0);
    motor4.setVelocityGain(1, 0);
    motor4.setTorqueEnable(true);

    motor5.setTorqueEnable(false);
    motor5.setVelocityLimit(15);
    motor5.setHomingOffset(6.632514615);
    motor5.setPositionGain(15, 0, 0);
    motor5.setVelocityGain(1, 0);
    motor5.setTorqueEnable(true);

    motor6.setTorqueEnable(false);
    motor6.setVelocityLimit(15);
    motor6.setHomingOffset(0);
    motor6.setPositionGain(15, 0, 0);
    motor6.setVelocityGain(1, 0);
    motor6.setTorqueEnable(true);

    // Link
    l1 = 0.0, l2 = sqrt(pow(264.0,2.0)+pow(30.0,2.0)), l3 = sqrt(pow(258.0,2.0)+pow(30.0,2.0)), l4 = 0.0, l5 = 123.0, l6 = 0.0;

    link0 << 0.0, 0.0, l1;
    link1 << 0.0, 0.0, l2;
    link2 << 0.0, 0.0, l3;
    link3 << 0.0, 0.0, l4;
    link4 << 0.0, 0.0, l5;    // Without End Effector
    link5 << 0.0, 0.0, l6;

    // Initialization
    initial_position_euler << 300.0, 0.0, 140.0, 0.0, M_PI, 0.0;
    target_position_euler = initial_position_euler;

    // Setup

    while(nh.ok())
    {
        // Theta
        t1 = (M_PI/180.0)*motor1.getPresentPosition(),
        t2 = (M_PI/180.0)*motor2.getPresentPosition(),
        t3 = (M_PI/180.0)*motor3.getPresentPosition(),
        t4 = (M_PI/180.0)*motor4.getPresentPosition(),
        t5 = (M_PI/180.0)*motor5.getPresentPosition(),
        t6 = (M_PI/180.0)*motor6.getPresentPosition();

        theta << t1, t2, t3, t4, t5, t6; 

        std::cout << "theta:" << std::endl << theta << std::endl << std::endl;

        // cos(theta), sin(theta)
        c1 = cos(t1);    s1 = sin(t1);
        c2 = cos(t2);    s2 = sin(t2);
        c3 = cos(t3);    s3 = sin(t3);
        c4 = cos(t4);    s4 = sin(t4);
        c5 = cos(t5);    s5 = sin(t5);
        c6 = cos(t6);    s6 = sin(t6);
        
        // Rotation
        rotation[0] = rotationZ(t1);
        rotation[1] = rotationY(t2);
        rotation[2] = rotationY(t3);
        rotation[3] = rotationZ(t4);
        rotation[4] = rotationY(t5);
        rotation[5] = rotationZ(t6);

        // Position
        now_position = rotation[0]*(link0+rotation[1]*(link1+rotation[2]*(link2+rotation[3]*(link3+rotation[4]*(link4+rotation[5]*link5)))));

        // Euler
        rotation_all = rotation[0]*rotation[1]*rotation[2]*rotation[3]*rotation[4]*rotation[5];

        e2 = acos(rotation_all(2,2));
        e1 = asin(rotation_all(1,2)/sin(e2));
        e3 = asin(rotation_all(2,1)/sin(e2));

        now_euler << e1, e2, e3;

        // Position Euler
        now_position_euler << now_position(0,0), now_position(1,0), now_position(2,0), now_euler(0,0), now_euler(1,0), now_euler(2,0);

        std::cout << "now_position_euler:" << std::endl << now_position_euler << std::endl << std::endl;

        // Translation Jacobian
        translation_jacobian(0,0) = cos(t1)*sin(t2)*l2 + cos(t1)*sin(t2+t3)*(l3+l4) + (cos(t1)*cos(t2+t3)*cos(t4)*sin(t5)-sin(t1)*sin(t4)*sin(t5)+cos(t1)*sin(t2+t3)*cos(t5))*(l5+l6);
        translation_jacobian(0,1) = sin(t1)*cos(t2)*l2 + sin(t1)*cos(t2+t3)*(l3+l4) + (sin(t1)*cos(t2+t3)*cos(t5)-sin(t1)*sin(t2+t3)*cos(t4)*sin(t5))*(l5+l6);
        translation_jacobian(0,2) = sin(t1)*cos(t2+t3)*(l3+l4) + (sin(t1)*cos(t2+t3)*cos(t5)-sin(t1)*sin(t2+t3)*cos(t4)*sin(t5))*(l5+l6);
        translation_jacobian(0,3) = (cos(t1)*cos(t4)*sin(t5)-sin(t1)*cos(t2+t3)*sin(t4)*sin(t5))*(l5+l6);
        translation_jacobian(0,4) = (cos(t1)*sin(t4)*cos(t5)+sin(t1)*cos(t2+t3)*cos(t4)*cos(t5)-sin(t1)*sin(t2+t3)*sin(t5))*(l5+l6);
        translation_jacobian(0,5) = 0.0;

        translation_jacobian(1,0) = sin(t1)*cos(t2)*l2 + sin(t1)*sin(t2+t3)*(l3+l4) + (cos(t1)*sin(t4)*sin(t5)+sin(t1)*cos(t2+t3)*cos(t4)*sin(t5)+sin(t1)*sin(t2+t3)*cos(t5))*(l5+l6);
        translation_jacobian(1,1) = cos(t1)*sin(t2)*l2-cos(t1)*cos(t2+t3)*(l3+l4) + (cos(t1)*sin(t2+t3)*cos(t4)*sin(t5)-cos(t1)*cos(t2+t3)*cos(t5))*(l5+l6);
        translation_jacobian(1,2) = -cos(t1)*cos(t2+t3)*(l3+l4) + (cos(t1)*sin(t2+t3)*cos(t4)*sin(t5)-cos(t1)*cos(t2+t3)*cos(t5))*(l5+l6);
        translation_jacobian(1,3) = (sin(t1)*cos(t4)*sin(t5)+cos(t1)*cos(t2+t3)*sin(t4)*sin(t5))*(l5+l6);
        translation_jacobian(1,4) = (sin(t1)*sin(t4)*cos(t5)-cos(t1)*cos(t2+t3)*cos(t4)*cos(t5)+cos(t1)*sin(t2+t3)*sin(t5))*(l5+l6);
        translation_jacobian(1,5) = 0.0;

        translation_jacobian(2,0) = 0.0;
        translation_jacobian(2,1) = -sin(t2)*l2 -sin(t2+t3)*(l3+l4) - (sin(t2+t3)*cos(t5)+cos(t2+t3)*cos(t4)*sin(t5))*(l5+l6);
        translation_jacobian(2,2) = -sin(t2+t3)*(l3+l4) - (sin(t2+t3)*cos(t5)+cos(t2+t3)*cos(t4)*sin(t5))*(l5+l6);
        translation_jacobian(2,3) = sin(t2+t3)*sin(t4)*sin(t5)*(l5+l6);
        translation_jacobian(2,4) = -(cos(t2+t3)*sin(t5)+sin(t2+t3)*cos(t4)*cos(t5))*(l5+t6);
        translation_jacobian(2,5) = 0.0;

        // Rotation Jacobian
        alternating_euler <<
        -sin(e2)*cos(e3), sin(e3), 0.0,
        sin(e2)*sin(e3) , cos(e3), 0.0,
        cos(e2)         , 0.0    , 1.0;

        alternating_rotation <<
        0.0, c1 , c1 , s1*sin(t2+t3) , c1*c4-s1*cos(t2+t3)*s4, c1*s4*s5+s1*sin(t2+t3)*c5+s1*cos(t2+t3)*c4*c5,
        0.0, s1 , s1 , -c1*sin(t2+t3), s1*c4-c1*cos(t2+t3)*s4, s1*s4*s5-c1*sin(t2+t3)*c5-c1*cos(t2+t3)*c4*s5,
        1.0, 0.0, 0.0, cos(t2+t3)    , sin(t2+t3)*s4         , cos(t2+t3)*c5-sin(t2+t3)*c4*s5               ;

        rotation_jacobian = alternating_euler.inverse()*alternating_rotation;

        // Jacobian
        jacobian <<
        translation_jacobian(0,0), translation_jacobian(0,1), translation_jacobian(0,2), translation_jacobian(0,3), translation_jacobian(0,4), translation_jacobian(0,5),
        translation_jacobian(1,0), translation_jacobian(1,1), translation_jacobian(1,2), translation_jacobian(1,3), translation_jacobian(1,4), translation_jacobian(1,5),
        translation_jacobian(2,0), translation_jacobian(2,1), translation_jacobian(2,2), translation_jacobian(2,3), translation_jacobian(2,4), translation_jacobian(2,5),
        rotation_jacobian(0,0)   , rotation_jacobian(0,1)   , rotation_jacobian(0,2)   , rotation_jacobian(0,3)   , rotation_jacobian(0,4)   , rotation_jacobian(0,5)   ,
        rotation_jacobian(1,0)   , rotation_jacobian(1,1)   , rotation_jacobian(1,2)   , rotation_jacobian(1,3)   , rotation_jacobian(1,4)   , rotation_jacobian(1,5)   ,
        rotation_jacobian(2,0)   , rotation_jacobian(2,1)   , rotation_jacobian(2,2)   , rotation_jacobian(2,3)   , rotation_jacobian(2,4)   , rotation_jacobian(2,5)   ; 

        std::cout << "jacbian:" << std::endl << jacobian << std::endl << std::endl;
        
        // Jacobian Inverse
        jacobian_inverse = jacobian.inverse();

        std::cout << "jacobian_inverse:" << std::endl << jacobian_inverse << std::endl << std::endl;

        // Angular Velocity
        theta_dot = kp*jacobian_inverse*(target_position_euler-now_position_euler);

        td1 = theta_dot(0,0);
        td2 = theta_dot(1,0);
        td3 = theta_dot(2,0);
        td4 = theta_dot(3,0);
        td5 = theta_dot(4,0);
        td6 = theta_dot(5,0);

        // Check
        double det = jacobian.determinant();
        Eigen::Matrix<double, 6, 6> I;
        I = jacobian_inverse*jacobian;
        std::cout << "det:" << std::endl << det << std::endl << std::endl;
        std::cout << "I:" << std::endl << I << std::endl << std::endl;

        // Safety
        for(int i=0; i<6; i++)
        {
            if(isnan(theta_dot(i,0)))
            {
                is_valid = false;
                break;
            }
        }
        if(fabs(det) < 0.001)
        {
            is_valid = false;
        }

        if(is_valid)
        {
            motor1.setGoalVelocity(td1);
            motor1.setGoalVelocity(td2);
            motor1.setGoalVelocity(td3);
            motor1.setGoalVelocity(td4);
            motor1.setGoalVelocity(td5);
            motor1.setGoalVelocity(td6);
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

        std_msgs::Bool msg;
        if(is_in_target())
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

        is_valid_old = is_valid;
    }

    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor4.setTorqueEnable(false);
    motor5.setTorqueEnable(false);
    motor6.setTorqueEnable(false);
    return 0;
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

bool is_in_target()
{
    if(
    fabs(now_position_euler(0,0) - target_position_euler(0,0)) < 0.01 &&
    fabs(now_position_euler(1,0) - target_position_euler(1,0)) < 0.01 &&
    fabs(now_position_euler(2,0) - target_position_euler(2,0)) < 0.01 &&
    fabs(now_position_euler(3,0) - target_position_euler(3,0)) < 0.01 &&
    fabs(now_position_euler(4,0) - target_position_euler(4,0)) < 0.01 &&
    fabs(now_position_euler(5,0) - target_position_euler(5,0)) < 0.01)
    {
        return true;
    }
    else
    {
        return false;
    }
}