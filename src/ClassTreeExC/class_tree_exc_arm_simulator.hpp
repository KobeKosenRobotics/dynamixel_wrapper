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

        // Joint
        void setJoint();
        void setParent(const int &c_, const int &j_);
            int getParentNumber(const int &c_, const int &j_);
        void setChildren(const int &c_, const int &j_);
            int getChildrenNumber(const int &c_, const int &j_);

        void update(const std_msgs::Float32MultiArray &angular_velocity_);
        std_msgs::Float32MultiArray getAngle();
};

// Constructor
TreeExCArmSimulator::TreeExCArmSimulator()
{
    _angle.data.resize(JOINT_NUMBER);
    _angular_velocity.data.resize(JOINT_NUMBER);

    setJoint();
}

// Joint
void TreeExCArmSimulator::setJoint()
{
    for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    {
        _joint[i].setJoint(i);
    }

    for(int c = 0; c < CHAIN_NUMBER; c++)
    {
        for(int j = 0; j < (JOINT_NUMBER+CHAIN_NUMBER); j++)
        {
            setParent(c, j);
            setChildren(c, j);
        }
    }
}

void TreeExCArmSimulator::setParent(const int &c_, const int &j_)
{
    if(j_ <= 0) return;
    if(!tree_property.getChainMatrix(c_, j_)) return;

    int parent_number_ = getParentNumber(c_, (j_-1));

    if(0 <= parent_number_)
    {
        _joint[j_].setParent(_joint+parent_number_);
    }
}

int TreeExCArmSimulator::getParentNumber(const int &c_, const int &j_)
{
    if(j_ < 0) return -1;

    if(tree_property.getChainMatrix(c_, j_))
    {
        return j_;
    }

    else
    {
        return getParentNumber(c_, (j_-1));
    }
}

void TreeExCArmSimulator::setChildren(const int &c_, const int &j_)
{
    if(JOINT_NUMBER <= j_) return;
    if(!tree_property.getChainMatrix(c_, j_)) return;

    int children_number_ = getChildrenNumber(c_, (j_+1));

    if(0 <= children_number_)
    {
        _joint[j_].setChildren(_joint+children_number_);
    }
}

int TreeExCArmSimulator::getChildrenNumber(const int &c_, const int &j_)
{
    if((JOINT_NUMBER+CHAIN_NUMBER) < j_) return -1;

    if(tree_property.getChainMatrix(c_, j_))
    {
        return j_;
    }

    else
    {
        return getChildrenNumber(c_, (j_+1));
    }
}

void TreeExCArmSimulator::update(const std_msgs::Float32MultiArray &angular_velocity_)
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