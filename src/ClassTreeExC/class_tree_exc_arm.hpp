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
    for(int i = 1; i < 6; i++)
    {
        _joint[i].setJoint(i);
        _joint[i].setParent(_joint+i-1);
        _joint[i].setChildren(_joint+i+1);
        _joint[i].setChildren(_joint+i+2);
        _joint[i].setChildren(_joint+i+3);
    }
    _joint[2].printJoint();

    Eigen::Matrix<double, 4, 4> mat;
    mat.setIdentity();
    std::cout << tree_base.adjoint(mat) << std::endl;
}


#endif