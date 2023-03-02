#ifndef CLASS_TREE_EXC_ARM_HPP
#define CLASS_TREE_EXC_ARM_HPP

#include "class_tree_exc_arm_property.hpp"
#include "class_tree_exc_joint.hpp"

class TreeExCArm
{
    private:
        TreeExCJoint _joint[JOINT_NUMBER+CHAIN_NUMBER];
    public:
        TreeExCArm();
        void printTest();
};

TreeExCArm::TreeExCArm()
{
}

void TreeExCArm::printTest()
{
    // _joint[0].setJoint(0, nullptr, (_joint+1*sizeof(TreeExCJoint)));
    // _joint[1].setJoint(1, (_joint+0*sizeof(TreeExCJoint)), (_joint+2*sizeof(TreeExCJoint)));
    // _joint[2].setJoint(2, (_joint+1*sizeof(TreeExCJoint)), (_joint+3*sizeof(TreeExCJoint)));
    // _joint[0].setJoint(0, nullptr, (_joint+1));
    // _joint[1].setJoint(1, (_joint), (_joint+2));
    // _joint[2].setJoint(2, (_joint+1), (_joint+3));
    // _joint[0].setJoint(0, -1, 1);
    // _joint[1].setJoint(1, 0, 2);
    // _joint[2].setJoint(2, 1, 3);
    // _joint[0].printJoint();
    // _joint[1].printJoint();
    // _joint[2].printJoint();
    // std::cout << (_joint+1*sizeof(TreeExCJoint)) << std::endl;

    for(int i = 1; i < 6; i++)
    {
        _joint[i].setJoint(i);
        _joint[i].setParent(_joint+i-1);
        _joint[i].setChildren(_joint+i+1);
        _joint[i].setChildren(_joint+i+2);
        _joint[i].setChildren(_joint+i+3);
    }
    _joint[2].printJoint();
}


#endif