#include "class_exc_arm_property.hpp"
#include "class_exc_arm.hpp"

#include "class_wait.hpp"
#include "class_random.hpp"

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

// Global
ExCArm exc_arm;
Wait wait;
Random random_number;
int step = 5;
int challenge = 0;
int success = 0;
Eigen::Matrix<double, 350, 6> reachable_target_pose;

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

// Function
void sequence()
{
    switch(step)
    {
    case 0:
        /* code */
        break;

    case 5:
        exc_arm.setCalculationMode(0);
        #ifdef DOF6
        exc_arm.setTargetAngle(0.0, 0.2, 0.7, 0.0, 0.2, 0.0);
        #endif
        #ifdef DOFN
        exc_arm.setTargetAngle(exc_arm_property.getInitialTargetAngle());
        #endif
        step = 10;
        break;

    case 10:
        // if(exc_arm.isInTargetAngle())
        if(exc_arm.isInTargetAngle())
        {
            step = 15;
        }
        break;

    case 15:
        exc_arm.setCalculationMode(2);
        // exc_arm.setTargetPose(400.0, -100.0, 300.0, 0.5, 1.0, 0.5);
        #ifdef COLLECT_RANDOM_TARGET
        target_pose.position.x    = random_number.getRandomNumber(200.0, 500.0, 1.0),
        target_pose.position.y    = random_number.getRandomNumber(-200.0, 200.0, 1.0),
        target_pose.position.z    = random_number.getRandomNumber(200.0, 500.0, 1.0),
        target_pose.orientation.z = random_number.getRandomNumber(-1.0, 1.0, 0.05),
        target_pose.orientation.y = random_number.getRandomNumber(1.0, 1.5, 0.05),
        target_pose.orientation.x = random_number.getRandomNumber(-1.0, 1.0, 0.05);
        #endif
        #ifndef COLLECT_RANDOM_TARGET
        // target_pose.position.x    = reachable_target_pose(challenge,0);
        // target_pose.position.y    = reachable_target_pose(challenge,1);
        // target_pose.position.z    = reachable_target_pose(challenge,2);
        // target_pose.orientation.z = reachable_target_pose(challenge,3);
        // target_pose.orientation.y = reachable_target_pose(challenge,4);
        // target_pose.orientation.x = reachable_target_pose(challenge,5);
        target_pose.position.x    = random_number.getRandomNumber(200.0, 500.0, 1.0),
        target_pose.position.y    = random_number.getRandomNumber(-200.0, 200.0, 1.0),
        target_pose.position.z    = random_number.getRandomNumber(200.0, 500.0, 1.0),
        target_pose.orientation.z = random_number.getRandomNumber(-1.0, 1.0, 0.05),
        target_pose.orientation.y = random_number.getRandomNumber(1.0, 1.5, 0.05),
        target_pose.orientation.x = random_number.getRandomNumber(-1.0, 1.0, 0.05);
        #endif
        exc_arm.setTargetPose(target_pose);

        step = 20;

        challenge++;

        wait.reset();
        exc_arm.measurementStart();

        break;

    case 20:
        if(!wait.isWaiting(1.2*(exc_arm.getDurationTime())))
        {
            exc_arm.measurementEnd();
            success++;
            step = 5;
        }
        else if(exc_arm.isSingularConfiguration() || !exc_arm.isWithinAngleLimit())
        {
            exc_arm.resetSingularConfiguration();
            exc_arm.resetWithinAngleLimit();
            step = 5;
        }

        break;

    default:
        break;
    }
};

void SCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(!exc_arm.getMotorEnable())
    {
        exc_arm.setSensorAngle(*msg);
    }
}

void MCA_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(exc_arm.getMotorEnable())
    {
        exc_arm.setSensorAngle(*msg);
    }
}

void target_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    // exc_arm.setTargetAngle(*msg);
}

void target_pose_cb(geometry_msgs::Pose::ConstPtr msg)
{
    // exc_arm.setTargetPose(*msg);
}

void motor_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    exc_arm.setMotorEnable(*msg);
}

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    exc_arm.setEmergencyStop(*msg);
}

void calculation_mode_cb(std_msgs::Int16::ConstPtr msg)
{
    // exc_arm.setCalculationMode(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExC_Calculator");
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

    reachable_target_pose <<
    441, 147, 472, 0.5, 1, 0.65,
    487, -149, 485, 0.05, 1.35, 0.9,
    471, 177, 281, 0.4, 1.45, -0.95,
    381, 134, 473, 0.5, 1.3, -1,
    490, 143, 461, -0.4, 1.45, -0.65,
    455, 179, 410, -0.8, 1.1, -0.65,
    401, -176, 437, 0.65, 1.1, 0.2,
    421, 94, 388, 0.45, 1.35, -0.9,
    434, 189, 373, -0.9, 1.5, -0.6,
    488, -20, 456, 0, 1.5, -0.55,
    315, -80, 499, -0.7, 1.5, 0.4,
    338, -40, 472, 0.8, 1.4, -0.15,
    439, -98, 453, -0.45, 1, -0.2,
    424, 44, 415, -0.2, 1.1, 0.45,
    451, 184, 381, 0.7, 1.5, 0.05,
    483, -140, 424, -0.45, 1.25, -0.85,
    481, -48, 454, -0.9, 1.4, 0.05,
    416, -25, 494, -0.1, 1.3, 0.95,
    468, -14, 471, -0.7, 1.25, 0.5,
    370, 19, 463, -0.95, 1.5, -0.1,
    444, -132, 422, 0.45, 1.2, 0.95,
    396, -187, 488, -0.05, 1.05, 0.05,
    407, -67, 462, -0.3, 1.05, -0.8,
    433, 170, 333, 0.95, 1.1, -0.45,
    360, -163, 487, -0.05, 1.25, -0.05,
    487, 37, 448, 0.25, 1.5, 0.85,
    398, -27, 472, -0.95, 1.45, -1,
    426, 108, 440, -0.4, 1.35, 0.2,
    488, 33, 500, 0.2, 1.2, 0.75,
    499, 135, 371, -0.75, 1.3, -0.9,
    472, 144, 394, -0.7, 1.15, 0.35,
    474, 31, 410, 0.45, 1.15, 0.75,
    479, -174, 453, -1, 1.25, 0.05,
    481, 33, 389, 0.65, 1.35, 0.35,
    457, 55, 476, 1, 1.45, 0.25,
    468, -165, 372, -0.55, 1.4, -0.45,
    367, 197, 492, -0.55, 1.15, 0.05,
    479, -107, 381, 0.45, 1, -0.25,
    460, 131, 366, 0.85, 1.5, 0.95,
    483, 37, 431, 0.55, 1.2, -0.2,
    474, 22, 248, 0.25, 1.45, -1,
    400, -193, 479, -0.35, 1.2, 0,
    477, -65, 349, -0.45, 1.2, 0.8,
    422, -56, 345, 0.9, 1.5, -0.15,
    437, -53, 457, -0.15, 1.2, -0.9,
    438, 98, 415, -0.1, 1.2, 0.8,
    451, 166, 439, -0.2, 1.3, 0.05,
    500, -61, 371, -0.65, 1.2, -0.25,
    484, -59, 356, -0.75, 1.25, -0.9,
    494, 177, 356, 0, 1.35, -0.55,
    449, 12, 429, -1, 1.15, -0.55,
    473, -117, 336, -0.4, 1.15, 1,
    356, -182, 482, 0.35, 1.5, -0.3,
    475, -112, 449, -0.7, 1.1, -0.2,
    473, -147, 433, 0.85, 1.25, 0.35,
    460, 33, 297, 0.4, 1.35, -0.7,
    489, 100, 368, 0.75, 1.2, 0,
    403, -173, 417, 0.45, 1.05, 0,
    486, 144, 395, 0, 1.25, -1,
    488, 84, 430, -0.4, 1.2, 0.55,
    445, -142, 431, 0.1, 1.4, 0.7,
    462, 79, 492, -0.15, 1.2, -0.75,
    402, 148, 422, 0.6, 1.45, -0.45,
    500, -122, 452, 0.75, 1.2, 0.35,
    433, -167, 399, -0.9, 1.35, 0.35,
    497, -53, 441, 0.95, 1.3, 0.55,
    434, 117, 488, 0.1, 1.2, -0.1,
    386, 166, 457, -0.5, 1.15, -0.15,
    425, -85, 427, -0.55, 1.1, 0.35,
    468, -145, 425, -0.85, 1.5, 0.25,
    461, -144, 362, -0.65, 1, 0.65,
    471, -132, 470, -0.45, 1.15, 0.55,
    363, -66, 453, -0.95, 1.5, 0.25,
    360, -107, 432, 0.7, 1.5, -0.1,
    430, 63, 499, 0.35, 1.25, -0.15,
    430, -16, 461, -0.2, 1.1, -0.7,
    460, -76, 364, 0.5, 1.05, -0.45,
    328, -138, 498, 0.6, 1.35, 0.05,
    392, -74, 444, 0.5, 1.2, -0.15,
    466, 43, 418, -0.1, 1.25, 0.75,
    427, -191, 362, 0.55, 1, 0,
    463, 180, 353, 0.85, 1.4, -0.1,
    388, 99, 407, -0.2, 1.4, 0.65,
    480, -168, 318, -0.05, 1.05, -0.1,
    434, -170, 432, 0.5, 1.45, -0.4,
    455, 200, 469, 1, 1.3, -0.3,
    455, 99, 384, 0.65, 1.5, -0.7,
    480, -145, 489, -0.25, 1.5, -0.25,
    415, 146, 415, 0.55, 1.05, -0.85,
    366, 100, 439, -0.3, 1.5, 0.6,
    451, 173, 304, -0.9, 1.35, -0.45,
    458, 197, 308, -0.5, 1.35, 0.25,
    441, -148, 448, -0.45, 1.35, -0.7,
    416, -117, 413, -0.9, 1.3, 0.3,
    394, -183, 480, 0.7, 1, 0.4,
    445, -197, 460, 0.65, 1.5, 0.8,
    425, 129, 421, -0.15, 1.3, 0.65,
    473, -163, 398, -0.2, 1.05, 0.85,
    433, 32, 488, -0.75, 1.1, -0.1,
    481, 179, 367, -0.4, 1.45, -0.05,
    494, 168, 390, 0.7, 1.4, -0.55,
    492, -1, 364, -0.6, 1.1, 0.15,
    342, -168, 408, 0.3, 1.5, -0.35,
    474, 137, 413, -0.8, 1.5, -0.7,
    409, -128, 386, -0.4, 1.35, 0.95,
    487, -37, 476, -0.5, 1.2, -1,
    414, -167, 491, -0.2, 1.35, 0.05,
    441, 193, 346, 0.8, 1.2, -0.75,
    481, -119, 418, -0.2, 1.45, -0.5,
    494, -59, 489, 0.9, 1.35, 0.75,
    499, -191, 251, -1, 1.4, 0.3,
    494, 193, 287, 0.1, 1.35, 0.9,
    485, -152, 417, -0.9, 1.45, -0.2,
    421, 45, 455, -0.25, 1.3, 0.65,
    469, 119, 388, 0.05, 1.45, 1,
    472, -9, 437, 0.2, 1.25, 0.6,
    481, -45, 332, 0.05, 1.5, -0.85,
    374, 120, 452, 0.05, 1.1, 0.65,
    336, 172, 480, 0.85, 1.15, 1,
    426, 174, 431, 0.05, 1.35, 0.95,
    496, -166, 390, -0.7, 1.3, -0.65,
    454, -149, 349, 0.3, 1.4, 0,
    366, 168, 473, 0.3, 1.45, -0.7,
    394, 134, 478, -0.55, 1.2, -0.05,
    483, -189, 379, -0.55, 1, 0.8,
    435, -123, 429, -0.15, 1.35, -0.15,
    483, -177, 490, -0.2, 1.05, 0.45,
    355, 121, 456, 0.45, 1.3, -0.95,
    470, 131, 374, -0.2, 1.4, -0.2,
    465, -64, 394, -0.1, 1.25, -0.55,
    494, 153, 265, 0.85, 1.35, -0.65,
    457, -84, 488, -0.45, 1.1, 0.1,
    374, 140, 473, 0.8, 1.5, 0.4,
    490, 87, 292, 0.3, 1.45, 0.8,
    500, -69, 497, -0.05, 1.2, -0.05,
    406, 159, 426, 0.55, 1.25, 1,
    316, 35, 494, -0.4, 1.45, 0.4,
    446, 119, 443, 0.35, 1.3, -0.7,
    464, -26, 436, 0.7, 1, 0.1,
    495, -111, 346, 0.65, 1.1, 0.5,
    393, -65, 495, -0.05, 1.45, -0.85,
    489, -166, 424, -0.35, 1.05, 0.65,
    424, 192, 483, -0.15, 1.2, -0.2,
    493, -34, 399, -0.45, 1.4, 0.85,
    391, -66, 484, -0.35, 1.1, 1,
    451, 20, 459, 0.3, 1.5, 0.35,
    449, -73, 481, 0.05, 1.45, -0.6,
    475, 117, 360, 0.7, 1.4, -0.7,
    486, 129, 253, -0.15, 1.15, 0.65,
    489, 157, 355, 0.4, 1.5, -0.95,
    485, 81, 230, -1, 1.45, 0,
    482, 148, 375, -0.95, 1.4, -0.5,
    467, 91, 414, -0.25, 1.25, -0.05,
    400, -50, 469, -0.75, 1.4, 0.45,
    397, 5, 500, 0.4, 1.05, -0.8,
    457, 71, 330, 0.8, 1.5, -0.55,
    451, -192, 394, -0.05, 1.05, -0.85,
    474, 12, 319, -0.3, 1.35, 0.7,
    439, -160, 431, 0.55, 1.2, 0.3,
    436, -72, 440, -0.55, 1.25, 0.4,
    404, -78, 499, -0.25, 1.35, -0.65,
    451, -93, 489, 0.6, 1.45, 0.8,
    482, 64, 382, -0.7, 1.3, -1,
    406, 51, 492, 0.7, 1.5, 0.05,
    469, 57, 453, -0.05, 1.1, 0.75,
    413, -150, 482, -0.5, 1.1, -0.9,
    426, -56, 470, 0.3, 1, 0.75,
    400, -111, 484, 0.25, 1.05, 0,
    374, -145, 493, 0.65, 1.2, 0.1,
    421, -65, 460, -0.55, 1.25, -0.05,
    497, -110, 475, 0.2, 1.2, 0.85,
    440, -176, 390, 0.6, 1.3, 0.6,
    397, -112, 418, 0.6, 1.45, -0.05,
    410, 156, 437, 0.15, 1.1, 0.45,
    489, -150, 377, 0.55, 1.5, 0.35,
    494, -148, 296, -0.15, 1, 0.75,
    449, -75, 425, -0.25, 1.3, -0.5,
    500, -111, 269, 0.95, 1.4, 0.25,
    494, 32, 413, -0.1, 1.15, 0.45,
    458, -178, 421, -0.65, 1.4, -0.55,
    447, -195, 459, -0.65, 1.45, -0.1,
    491, -117, 379, 0.55, 1.2, 0.2,
    465, -127, 321, 0.6, 1.15, -0.05,
    498, -90, 355, 0.2, 1.3, -0.4,
    471, -94, 404, -0.55, 1.3, -0.1,
    442, 149, 462, -0.5, 1.5, -0.9,
    393, 58, 451, 1, 1.45, -0.4,
    393, 155, 477, -1, 1.45, -1,
    500, 66, 417, 0.25, 1.4, -0.2,
    373, -33, 458, -0.75, 1.25, 0.3,
    488, -199, 401, -0.95, 1.2, -0.1,
    387, 14, 471, 0.25, 1.25, -1,
    493, 161, 407, -0.8, 1.25, -0.3,
    442, -57, 486, 0.55, 1.3, 0.1,
    470, 8, 394, 0.2, 1.25, -0.15,
    371, -89, 455, 0.25, 1.2, -0.7,
    496, -64, 372, -0.45, 1.3, 0.25,
    372, 120, 480, -0.1, 1.25, -0.9,
    491, -66, 476, 0.15, 1.05, 1,
    413, 103, 351, 0.6, 1.2, -0.75,
    417, -108, 445, -0.3, 1.5, -0.45,
    365, -130, 469, -0.5, 1, 0.95,
    473, -197, 449, 0.85, 1.3, 0.8,
    440, -187, 463, 0.6, 1.35, 0.6,
    420, 171, 441, 0.3, 1, 0.85,
    488, -79, 469, 0.05, 1.45, 0.45,
    333, -95, 472, -0.35, 1.35, -1,
    405, -183, 440, 0.8, 1.15, 0.9,
    482, 132, 403, -0.45, 1, 0.4,
    449, 68, 410, 0.65, 1.3, -0.5,
    400, -19, 471, 0.5, 1.15, -0.4,
    463, 45, 318, 1, 1.5, -0.3,
    498, 57, 489, 0.4, 1.15, 0.7,
    427, 104, 485, 0.6, 1.05, -0.2,
    409, 8, 463, 0.75, 1.1, -0.2,
    366, -168, 463, -0.5, 1, 0.75,
    465, -143, 490, -0.5, 1.35, 0.15,
    439, 195, 427, 0.7, 1.1, -0.1,
    456, 137, 381, 0.8, 1.3, -0.25,
    445, -79, 436, 0.25, 1.3, 0.25,
    453, -36, 492, 0.75, 1.2, 0.9,
    440, -159, 422, 0.25, 1.15, -0.25,
    497, -56, 418, -0.25, 1.35, 0.25,
    385, 196, 483, 0.7, 1.5, 0.15,
    // 492, 114, 493, 0.05, 1.2, 0.65,
    397, -72, 467, -0.95, 1.1, 0.2,
    483, -175, 357, -0.85, 1.5, -0.2,
    361, 101, 498, 0.1, 1.5, 0.7,
    492, -98, 379, -0.7, 1.25, -0.9,
    413, 159, 415, 0.55, 1.05, -0.55,
    484, 167, 339, -0.55, 1.25, -0.35,
    397, 171, 489, 0.3, 1.45, -0.45,
    428, 102, 464, 0.5, 1.45, -0.35,
    450, 82, 361, -0.9, 1.35, -0.2,
    422, 109, 455, 0.5, 1.2, 0.6,
    476, -190, 361, -0.65, 1.15, 0.55,
    491, -173, 369, 0.75, 1.35, 0,
    358, -145, 456, 0.45, 1.5, -0.2,
    401, 190, 483, 0.35, 1.4, -0.9,
    432, -116, 487, -0.15, 1.3, 0.3,
    477, -135, 438, -0.15, 1.05, -0.2,
    398, -127, 429, 0.1, 1.4, -0.45,
    457, -88, 458, -0.4, 1.1, -0.4,
    472, -117, 469, -0.35, 1.35, -0.8,
    492, -168, 438, 0.4, 1.3, -0.1,
    480, 0, 403, 0.05, 1.15, -0.15,
    478, 18, 495, 0.45, 1.45, 0.75,
    481, 195, 329, 0.45, 1.3, -0.6,
    412, 26, 388, -0.95, 1.45, 0.2,
    491, 97, 311, 0.3, 1.2, -0.9,
    460, 75, 488, 0.15, 1.1, -0.15,
    441, -46, 353, 0.15, 1.15, -0.85,
    450, -73, 421, 0.15, 1.45, -0.1,
    497, 106, 410, -0.2, 1.5, -1,
    360, -120, 487, 0.05, 1.35, -0.25,
    494, 154, 469, -0.25, 1.35, -0.5,
    485, -171, 399, 0.7, 1.15, 0.7,
    496, 139, 464, -0.4, 1, -0.05,
    498, -154, 382, -0.25, 1.1, 0.65,
    497, -122, 307, -0.65, 1.5, 0.65,
    392, -187, 397, 0.05, 1, -0.35,
    460, 185, 374, -0.45, 1.15, -0.1,
    465, 12, 439, -0.7, 1.15, 0.1,
    491, -125, 369, -0.2, 1, 0.8,
    467, -139, 385, -0.2, 1.25, 0.6,
    472, -38, 449, 0.9, 1.45, 0.05,
    493, 160, 470, 0.15, 1.2, -0.15,
    457, 36, 348, -1, 1.35, -0.1,
    483, -193, 331, -0.4, 1.05, -0.6,
    459, -161, 317, -0.05, 1.1, -0.55,
    471, -119, 470, 0.1, 1.3, -0.45,
    441, 84, 463, -0.9, 1, -0.1,
    406, -170, 424, -0.65, 1.35, -0.75,
    427, 81, 363, 0.95, 1.4, -0.35,
    436, 107, 311, 0.65, 1.45, -0.65,
    494, 55, 471, -0.4, 1.2, -0.55,
    420, -171, 499, 0.75, 1.25, 0.55,
    468, -183, 479, 0.05, 1.05, -0.35,
    498, 113, 395, 0.1, 1, -0.6,
    487, -73, 429, -0.45, 1.05, -0.05,
    437, 50, 495, -0.9, 1.5, -0.4,
    439, 69, 467, 0.1, 1.2, -0.6,
    447, 42, 476, -0.2, 1.15, 0.35,
    355, -73, 484, 0.5, 1.05, -0.35,
    442, 64, 416, 0.4, 1.5, -0.95,
    438, -196, 426, 0.45, 1.15, 0.6,
    481, -169, 307, -0.05, 1.4, -0.55,
    461, -191, 424, -0.7, 1.4, -0.6,
    423, -193, 318, 0.35, 1.25, -0.35,
    497, 0, 410, -0.35, 1, -0.6,
    435, -88, 478, 1, 1.45, 0.15,
    491, 178, 443, 0.05, 1.4, -0.05,
    454, -41, 453, 0.45, 1, -0.7,
    481, -142, 428, 0.3, 1.5, -0.15,
    455, 77, 432, 0, 1.1, 0.75,
    493, 27, 360, -0.5, 1.25, 0.3,
    480, 111, 429, 0.05, 1.3, 0.9,
    473, -22, 469, 0.9, 1, 0.3,
    491, 93, 469, 0.2, 1.4, 0.25,
    461, 111, 398, -0.2, 1.3, -0.05,
    436, -17, 466, -0.2, 1.45, 0.2,
    371, -160, 440, -0.7, 1.35, -1,
    496, 179, 474, -0.35, 1.05, -0.9,
    440, -125, 333, -0.8, 1.3, 0.6,
    442, -166, 459, -0.95, 1, 0.5,
    449, -127, 456, 0.55, 1.45, -0.4,
    392, 177, 457, -0.25, 1.5, -0.5,
    437, -6, 483, 0.75, 1.15, -0.4,
    490, -86, 239, -0.2, 1.45, -1,
    487, 145, 470, -0.1, 1.1, -0.25,
    359, 177, 491, -0.15, 1.5, 0,
    491, -192, 252, 0.1, 1.45, -0.25,
    426, -163, 432, 0.1, 1.1, -0.5,
    354, 194, 449, 0, 1.45, 0.4,
    453, -139, 486, -0.65, 1.1, -0.15,
    421, -23, 489, -0.45, 1.25, -0.5,
    436, 54, 495, 0, 1.2, 0.9,
    397, 196, 476, -0.1, 1.25, 0.3,
    387, -161, 490, 0.4, 1.15, 0.05,
    446, 131, 378, 0.3, 1.2, -0.6,
    487, 64, 472, 0.55, 1.25, 0.2,
    475, 170, 461, 1, 1.35, 0.8,
    476, -176, 277, -0.1, 1.4, -0.7,
    467, -92, 494, 0.45, 1.1, 1,
    456, 117, 472, 0.75, 1.45, -0.4,
    469, -140, 384, -0.55, 1.3, -0.3,
    386, 158, 450, 0.05, 1.4, -0.85,
    494, 115, 424, 0.45, 1.2, 0.8,
    287, -168, 471, -0.2, 1.3, -0.75,
    487, -136, 345, -0.4, 1.3, -0.65,
    446, -163, 499, -0.6, 1.2, -0.1,
    496, 191, 298, 0.3, 1.3, -0.65,
    463, -21, 372, -0.75, 1.45, 0.15,
    423, -154, 323, -0.4, 1.3, 0.95,
    495, -196, 367, -0.05, 1.05, 0.95,
    447, -98, 424, 1, 1.2, 0.5,
    489, 100, 287, 0.55, 1.1, -0.45,
    432, 119, 434, -0.75, 1.5, -0.5,
    416, -49, 483, -0.15, 1.5, -0.3,
    419, -143, 442, -1, 1.25, -0.1,
    385, 163, 480, -0.2, 1.3, -0.05,
    368, 163, 441, -0.45, 1.35, 0.3,
    489, -124, 358, 0.75, 1.05, 0.4,
    453, 57, 447, 0.6, 1.5, -0.2,
    431, -87, 492, -0.25, 1.1, 0.8,
    491, 108, 331, -0.05, 1, 0.45,
    483, -62, 400, -0.1, 1.4, 0.7,
    395, -76, 481, 0.3, 1.2, -0.3,
    482, -105, 426, -0.1, 1.3, -0.4,
    471, 120, 497, 0.05, 1.15, 0.25,
    362, -73, 492, -0.15, 1.45, 0.75;


    while(nh.ok())
    {
        // exc_arm.print();

        sequence();

        if(exc_arm.getMotorEnable())
        {
            CSAV_pub.publish(exc_arm.getMotorAngularVelocityZero());
            CMAV_pub.publish(exc_arm.getMotorAngularVelocity());
        }
        else
        {
            CSAV_pub.publish(exc_arm.getMotorAngularVelocity());
            CMAV_pub.publish(exc_arm.getMotorAngularVelocityZero());
        }

        if(success > 100) break;
        else if(challenge >= 350) break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "DOF" << JOINT_NUMBER << std::endl;

    return 0;
}