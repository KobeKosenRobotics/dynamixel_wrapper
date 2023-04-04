#include "class_tree_exc_arm.hpp"

// Global
TreeExCArm tree_arm;

// Publisher
std_msgs::Float32MultiArray CSAV;
std_msgs::Float32MultiArray CMAV;

// Subscriber
std_msgs::Float32MultiArray SCA;
std_msgs::Float32MultiArray MCA;
std_msgs::Float32MultiArray target_angle;
geometry_msgs::Pose target_pose;
std_msgs::Bool motor_enable;
std_msgs::Bool exc_enable;
std_msgs::Bool emergency_stop;

void SCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    // if(!tree_arm.getMotorEnable())
    {
        tree_arm.setSensorAngle(msg);
    }
}

void MCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    // if(tree_arm.getMotorEnable())
    {
        tree_arm.setSensorAngle(msg);
    }
}

void target_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    tree_arm.setTargetAngle(msg);
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    tree_arm.setTargetPose(msg);
}

void motor_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    tree_arm.setMotorEnable(msg);
}

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    tree_arm.setEmergencyStop(msg);
}

void calculation_mode_cb(std_msgs::Int16::ConstPtr msg)
{
    tree_arm.setCalculationMode(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TreeCalculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher CSAV_pub = nh.advertise<std_msgs::Float32MultiArray>("CSAV", 100);
    CSAV.data.resize(JOINT_NUMBER);
    ros::Publisher CMAV_pub = nh.advertise<std_msgs::Float32MultiArray>("CMAV", 100);
    CMAV.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber SCA_sub = nh.subscribe<std_msgs::Float32MultiArray>("SCA", 100, SCA_cb);
    SCA.data.resize(JOINT_NUMBER);
    ros::Subscriber MCA_sub = nh.subscribe<std_msgs::Float32MultiArray>("MCA", 100, MCA_cb);
    MCA.data.resize(JOINT_NUMBER);
    ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_angle", 10, target_angle_cb);
    target_angle.data.resize(JOINT_NUMBER);
    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 10, target_pose_cb);
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("motor_enable", 10, motor_enable_cb);
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);
    ros::Subscriber calculation_mode_sub = nh.subscribe<std_msgs::Int16>("calculation_mode", 10, calculation_mode_cb);

    while(nh.ok())
    {
        tree_arm.print();

        CSAV_pub.publish(tree_arm.getMotorAngularVelocity());

        if(tree_arm.getMotorEnable())
        {
            CMAV_pub.publish(tree_arm.getMotorAngularVelocity());
        }
        else
        {
            CMAV_pub.publish(tree_arm.getMotorAngularVelocityZero());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}