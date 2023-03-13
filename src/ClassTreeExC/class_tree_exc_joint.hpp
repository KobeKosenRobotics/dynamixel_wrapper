#ifndef CLASS_TREE_EXC_JOINT_HPP
#define CLASS_TREE_EXC_JOINT_HPP

#include "class_tree_exc_arm_property.hpp"

class TreeExCJoint
{
    private:
        // Family
        int _joint;
        TreeExCJoint* _parent_joint = nullptr;
        std::vector<TreeExCJoint*> _children_joint{nullptr};
        int _children_number = 0;

        // Parameter
        Eigen::Matrix<double, 3, 1> _q, _v, _w;
        Eigen::Matrix<double, 6, 1> _xi;
        Eigen::Matrix<double, 4, 4> _gsj_zero;

        double _theta, _cos_theta, _sin_theta, _v_theta;

        // Matrix
        Eigen::Matrix<double, 3, 3> _exp_w_hat_theta;
        Eigen::Matrix<double, 4, 4> _exp_xi_hat_theta;
        Eigen::Matrix<double, 6, 1> _xi_dagger;

        int _minimum_joint;

    public:
        // Constructor
        TreeExCJoint();

        // Family
        void setJoint(int joint_);
            Eigen::Matrix<double, 3, 1> getQ();
            Eigen::Matrix<double, 6, 1> getXi();
            Eigen::Matrix<double, 4, 4> getGsjZero();
        void setParent(TreeExCJoint *parent_joint_);
        void setChildren(TreeExCJoint *children_joint_);
        void printJoint();

        // Theta
        void updateTheta(double &theta_);

        // Matrix
        Eigen::Matrix<double, 4, 4> getExpXiHatTheta();
            Eigen::Matrix<double, 3, 3> getExpWHatTheta();

        Eigen::Matrix<double, 4, 4> getGsjTheta();
            Eigen::Matrix<double, 4, 4> getGsjThetaRecursion();
        Eigen::Matrix<double, 4, 4> getChildrenExpXiHatTheta(int minimum_joint_);
            Eigen::Matrix<double, 4, 4> getChildrenExpXiHatThetaRecursion();

};

TreeExCJoint::TreeExCJoint()
{
    updateTheta(_theta);
}

// Family
void TreeExCJoint::setJoint(int joint_)
{
    _joint = joint_;

    _q = getQ();
    _v = tree_property.getV(_joint);
    _w = tree_property.getW(_joint);

    if(_v.norm() == 0)
    {
        _v = -_w.cross(_q);
    }

    getXi();

    getGsjZero();
}

Eigen::Matrix<double, 3, 1> TreeExCJoint::getQ()
{
    if(_parent_joint == nullptr)
    {
        return tree_property.getLink(_joint);
    }
    return _parent_joint->_q+tree_property.getLink(_joint);
}

Eigen::Matrix<double, 6, 1> TreeExCJoint::getXi()
{
    _xi << -_w.cross(_q), _w;

    return _xi;
}

Eigen::Matrix<double, 4, 4> TreeExCJoint::getGsjZero()
{
    _gsj_zero <<
    tree_base.getIdentity3(), _q,
    0.0, 0.0, 0.0, 1.0;

    return _gsj_zero;
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

void TreeExCJoint::printJoint()
{
    std::cout << "joint: " << _joint << std::endl;
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
}

// Parameter
void TreeExCJoint::updateTheta(double &theta_)
{
    _theta = theta_;
    _cos_theta = cos(theta_);
    _sin_theta = sin(theta_);
    _v_theta = 1 - _cos_theta;
}

// Parameter
Eigen::Matrix<double, 4, 4> TreeExCJoint::getExpXiHatTheta()
{
    updateTheta(_theta);
    getExpWHatTheta();

    _exp_xi_hat_theta <<
    _exp_w_hat_theta, ((tree_base.getIdentity3() - _exp_w_hat_theta) * (_w.cross(_v))) + (_w*_w.transpose()) * _v*_theta,
    0.0, 0.0, 0.0, 1.0;

    return _exp_xi_hat_theta;
}

Eigen::Matrix<double, 3, 3> TreeExCJoint::getExpWHatTheta()
{
    _exp_w_hat_theta(0,0) = pow(_w(0,0),2)*_v_theta + _cos_theta;
    _exp_w_hat_theta(0,1) = _w(0,0)*_w(1,0)*_v_theta - _w(2,0)*_sin_theta;
    _exp_w_hat_theta(0,2) = _w(0,0)*_w(2,0)*_v_theta + _w(1,0)*_sin_theta;

    _exp_w_hat_theta(1,0) = _w(0,0)*_w(1,0)*_v_theta + _w(2,0)*_sin_theta;
    _exp_w_hat_theta(1,1) = pow(_w(1,0),2)*_v_theta + _cos_theta;
    _exp_w_hat_theta(1,2) = _w(1,0)*_w(2,0)*_v_theta - _w(0,0)*_sin_theta;

    _exp_w_hat_theta(2,0) = _w(0,0)*_w(2,0)*_v_theta - _w(1,0)*_sin_theta;
    _exp_w_hat_theta(2,1) = _w(1,0)*_w(2,0)*_v_theta + _w(0,0)*_sin_theta;
    _exp_w_hat_theta(2,2) = pow(_w(2,0),2)*_v_theta + _cos_theta;

    return _exp_w_hat_theta;
}

Eigen::Matrix<double, 4, 4> TreeExCJoint::getGsjTheta()
{
    return getGsjThetaRecursion()*_gsj_zero;
}

Eigen::Matrix<double, 4, 4> TreeExCJoint::getGsjThetaRecursion()
{
    if(_parent_joint == nullptr)
    {
        std::cout << std::endl;
        return getExpXiHatTheta();
    }
    if(_joint >= JOINT_NUMBER)
    {
        return _parent_joint->getGsjThetaRecursion();
    }

    return _parent_joint->getGsjThetaRecursion()*getExpXiHatTheta();
}

Eigen::Matrix<double, 4, 4> TreeExCJoint::getChildrenExpXiHatTheta(int minimum_joint_)
{
    if(_joint < JOINT_NUMBER) std::cout << "ERROR: _joint < JOINT_NUMBER" << std::endl;

    _minimum_joint = minimum_joint_;
    _parent_joint->_minimum_joint = _minimum_joint;

    return getChildrenExpXiHatThetaRecursion()*_gsj_zero;
}

Eigen::Matrix<double, 4, 4> TreeExCJoint::getChildrenExpXiHatThetaRecursion()
{
    if(_joint == _minimum_joint)
    {
        return getExpXiHatTheta();
    }

    _parent_joint->_minimum_joint = _minimum_joint;

    if(_joint >= JOINT_NUMBER)
    {
        return _parent_joint->getChildrenExpXiHatThetaRecursion();
    }

    return _parent_joint->getChildrenExpXiHatThetaRecursion()*getExpXiHatTheta();
}

#endif