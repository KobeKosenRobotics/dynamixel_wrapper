#ifndef CLASS_TREE_EXC_ARM_SIMULATOR_HPP
#define CLASS_TREE_EXC_ARM_SIMULATOR_HPP

#include "class_tree_exc_arm_property.hpp"
#include "class_tree_exc_joint_simulator.hpp"

class TreeExCArmSimulator
{
    private:
        TreeExCJointSimulator _joint[JOINT_NUMBER+CHAIN_NUMBER];
        std_msgs::Float32MultiArray _angle, _angular_velocity;

        void tfBroadcaster();

    public:
        // Constructor
        TreeExCArmSimulator();

        void update(std_msgs::Float32MultiArray &angular_velocity_);
        std_msgs::Float32MultiArray getAngle();
};

TreeExCArmSimulator::TreeExCArmSimulator()
{
}

void TreeExCArmSimulator::update(std_msgs::Float32MultiArray &angular_velocity_)
{
    _angular_velocity = angular_velocity_;

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _joint[i].angularVelocity2Angle(_angular_velocity.data[i]);
    }

    tfBroadcaster();
}

std_msgs::Float32MultiArray TreeExCArmSimulator::getAngle()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _angle.data[i] = _joint[i].getSimulationAngle();
    }
    return _angle;
}

void TreeExCArmSimulator::tfBroadcaster()
{
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Joint 0
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";
    transformStamped.child_frame_id = tree_property.getJointName(0);
    transformStamped.transform.translation.x = _joint[0].getSimulationLink(0);
    transformStamped.transform.translation.y = _joint[0].getSimulationLink(1);
    transformStamped.transform.translation.z = _joint[0].getSimulationLink(2);

    q.setRPY(_joint[0].getSimulationAngle(0), _joint[0].getSimulationAngle(1), _joint[0].getSimulationAngle(2));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Joint 1 ~
    for(int i = 1; i < (JOINT_NUMBER+CHAIN_NUMBER); i++)
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = tree_property.getJointName(_joint[i].getParentJoint());
        transformStamped.child_frame_id = tree_property.getJointName(i);
        transformStamped.transform.translation.x = _joint[i].getSimulationLink(0);
        transformStamped.transform.translation.y = _joint[i].getSimulationLink(1);
        transformStamped.transform.translation.z = _joint[i].getSimulationLink(2);

        q.setRPY(_joint[i].getSimulationAngle(0), _joint[i].getSimulationAngle(1), _joint[i].getSimulationAngle(2));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }
}

#endif