#ifndef CLASS_TREE_EXC_JOINT_HPP
#define CLASS_TREE_EXC_JOINT_HPP

#include "class_tree_exc_arm_property.hpp"

class TreeExCJoint
{
    private:
        int _joint;
        TreeExCJoint* _parent_joint = nullptr;
        // TreeExCJoint* _children_joint = nullptr;
        std::vector<TreeExCJoint*> _children_joint{nullptr};

        // Eigen::Matrix<double, 3, 1> _q, _v, _w;

        int _children_number = 0;
    public:
        TreeExCJoint();
        void setJoint(int joint_);
        void setParent(TreeExCJoint *parent_joint_);
        void setChildren(TreeExCJoint *children_joint_);
        // template<class Head, class... Tail> void setChildren(Head *head, Tail... *tail);
        void printJoint();

};

TreeExCJoint::TreeExCJoint()
{
}

void TreeExCJoint::setJoint(int joint_)
{
    _joint = joint_;
}

void TreeExCJoint::setParent(TreeExCJoint *parent_joint_)
{
    _parent_joint = parent_joint_;
}

void TreeExCJoint::setChildren(TreeExCJoint *children_joint_)
{
    _children_joint[_children_number] = children_joint_;
    _children_number++;
}

// template<class Head, class... Tail>
// void TreeExCJoint::setChildren(Head *head, Tail... *tail)
// {
//     // // int _count, _parent_joint, _head, _children_joint;
//     // if(_count == 0)
//     // {
//     //     // _joint = head;

//     //     // _q = exc_arm_property.getQ(joint_);
//     //     // // _v = exc_arm_property.getV(joint);
//     //     // _w = exc_arm_property.getW(joint_);
//     //     // _v = -_w.cross(_q);

//     //     // _eye3.setIdentity();
//     // }
//     // else if(_count == 1)
//     // {
//     //     // std::cout << head << std::endl;
//     //     // std::cout << &head << std::endl;
//     //     // _parent_joint = head;
//     // }
//     // else
//     // {
//     //     // _children_joint+sizeof(TreeExCJoint)*(_count-2) = head;
//     // }
//     (_children_joint+_count) = head;

//     _count++;

//     setChildren(std::forward<Tail>(tail)...);
// }

void TreeExCJoint::printJoint()
{
    std::cout << "joint: " << _joint << std::endl;
    // std::cout << sizeof(TreeExCJoint) << std::endl;
    std::cout << "parent: " << _parent_joint->_joint << std::endl;
    for(int i = 0; i < _children_number; i++)
    {
        if(i == 0)
        {
            std::cout << "children: ";
        }
        std::cout << _children_joint[i]->_joint << " ";
        if(i == _children_number-1)
        {
            std::cout << std::endl;
        }
    }
    // std::cout << "children: " << _children_joint->_joint << std::endl;
}




#endif