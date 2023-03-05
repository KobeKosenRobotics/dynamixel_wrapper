#ifndef CLASS_TREE_EXC_JOINT_SIMULATOR_HPP
#define CLASS_TREE_EXC_JOINT_SIMULATOR_HPP

#include "class_tree_exc_arm_property.hpp"

class TreeExCJointSimulator
{
    private:
        // Property
        int _joint;
        Eigen::Matrix<double, 3, 1> _w;
        TreeExCJointSimulator* _parent_joint = nullptr;
        std::vector<TreeExCJointSimulator*> _children_joint{nullptr};
        int _children_number = 0;

        // Simulator
        bool _is_first_simulation = true;
        ros::Time _start_time_simulation, _end_time_simulation;
        double _simulation_angular_velocity, _simulation_angle = 0.0;

    public:
        // Constructor
        TreeExCJointSimulator();

        // Property
        void setJoint(int joint_);
        void setParent(TreeExCJointSimulator *parent_joint_);
        void setChildren(TreeExCJointSimulator *children_joint_);

        // Simulator
        void angularVelocity2Angle(double &angular_velocity_);
        double getSimulationAngle();
        double getSimulationAngle(int axis_);
};

// Constructor
TreeExCJointSimulator::TreeExCJointSimulator()
{
}

// Property
void TreeExCJointSimulator::setJoint(int joint_)
{
    _joint = joint_;
    _w = tree_property.getW(_joint);
}

void TreeExCJointSimulator::setParent(TreeExCJointSimulator *parent_joint_)
{
    _parent_joint = parent_joint_;
}

void TreeExCJointSimulator::setChildren(TreeExCJointSimulator *children_joint_)
{
    _children_joint[_children_number] = children_joint_;
    _children_number++;
}

// Simulator
void TreeExCJointSimulator::angularVelocity2Angle(double &angular_velocity_)
{
    _simulation_angular_velocity = angular_velocity_;

    if(_is_first_simulation)
    {
        _is_first_simulation = false;
        _start_time_simulation = ros::Time::now();
    }
    _end_time_simulation = ros::Time::now();
    _simulation_angle += (_end_time_simulation-_start_time_simulation).toSec()*_simulation_angular_velocity;
    _start_time_simulation = _end_time_simulation;
}

double TreeExCJointSimulator::getSimulationAngle()
{
    return _simulation_angle;
}

double TreeExCJointSimulator::getSimulationAngle(int axis_)
{
    return _w(axis_,0)*_simulation_angle;
}
#endif