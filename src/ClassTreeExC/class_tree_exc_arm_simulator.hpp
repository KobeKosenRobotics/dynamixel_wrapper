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

        void setJoint();

        void update(std_msgs::Float32MultiArray &angular_velocity_);
        std_msgs::Float32MultiArray getAngle();
};

TreeExCArmSimulator::TreeExCArmSimulator()
{
    _angle.data.resize(JOINT_NUMBER);
    _angular_velocity.data.resize(JOINT_NUMBER);

    setJoint();
}

void TreeExCArmSimulator::setJoint()
{
    for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    {
        _joint[i].setJoint(i);
    }
    // Chain Matrix
    // - 0- 1- 2- 3- 4- 5- 6- 7-14
    //       - 8- 9-10-11-12-13-15
    _joint[0].setChildren(_joint+1);

    _joint[1].setParent(_joint+0);
    _joint[1].setChildren(_joint+2);
    _joint[1].setChildren(_joint+8);

    _joint[2].setParent(_joint+1);
    _joint[2].setChildren(_joint+3);

    _joint[3].setParent(_joint+2);
    _joint[3].setChildren(_joint+4);

    _joint[4].setParent(_joint+3);
    _joint[4].setChildren(_joint+5);

    _joint[5].setParent(_joint+4);
    _joint[5].setChildren(_joint+6);

    _joint[6].setParent(_joint+5);
    _joint[6].setChildren(_joint+7);

    _joint[7].setParent(_joint+6);
    _joint[7].setChildren(_joint+14);

    _joint[8].setParent(_joint+1);
    _joint[8].setChildren(_joint+9);

    _joint[9].setParent(_joint+8);
    _joint[9].setChildren(_joint+10);

    _joint[10].setParent(_joint+9);
    _joint[10].setChildren(_joint+11);

    _joint[11].setParent(_joint+10);
    _joint[11].setChildren(_joint+12);

    _joint[12].setParent(_joint+11);
    _joint[12].setChildren(_joint+13);

    _joint[13].setParent(_joint+12);
    _joint[13].setChildren(_joint+15);

    _joint[14].setParent(_joint+7);

    _joint[15].setParent(_joint+13);
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
        std::cout << _joint[i].getSimulationLink(0) << "    " << _joint[i].getSimulationLink(1) << "    " << _joint[i].getSimulationLink(2) << "    " << std::endl;

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