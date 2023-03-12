#include "class_tree_exc_arm_simulator.hpp"

// Global
TreeExCArmSimulator tree_simulator;

// Publisher
std_msgs::Float32MultiArray SCA;

// Subscriber
std_msgs::Float32MultiArray CSAV;

// Function
void CSAV_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    CSAV = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TreeSimulator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher SCA_pub = nh.advertise<std_msgs::Float32MultiArray>("SCA", 100);
    SCA.data.resize(JOINT_NUMBER);

    // Subscriber
    ros::Subscriber CSAV_sub = nh.subscribe<std_msgs::Float32MultiArray>("CSAV", 100, CSAV_cb);
    CSAV.data.resize(JOINT_NUMBER);

    while(nh.ok())
    {
        tree_simulator.update(CSAV);

        SCA_pub.publish(tree_simulator.getAngle());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}