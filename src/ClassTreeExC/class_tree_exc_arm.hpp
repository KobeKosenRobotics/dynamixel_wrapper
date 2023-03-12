#ifndef CLASS_TREE_EXC_ARM_HPP
#define CLASS_TREE_EXC_ARM_HPP

#include "class_tree_exc_arm_property.hpp"
#include "class_tree_exc_joint.hpp"

class TreeExCArm
{
    private:
        TreeExCJoint _joint[JOINT_NUMBER+CHAIN_NUMBER];

        // Bool
        bool _motor_enable = false;
        bool _emergency_stop = false;

        // Calculation Mode    0(else):Angle, 1:ExC
        int _calculation_mode = 0, _calculation_mode_old = 0;

        // Motor
        Eigen::Matrix<double, JOINT_NUMBER, 1> _sensor_angle, _motor_angular_velocity;

        // Angle Operating
        Eigen::Matrix<double, JOINT_NUMBER, 1> _target_angle;

        // Forward Kinematics
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> _pose;

        // Inverse Kinematics
        double _proportional_gain = 20.0;
        Eigen::Matrix<double, 6, 1> _target_object_pose;
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> _target_pose;

        // Linear Interpolation
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 300;    // _liner_velocity[mm/s]    // Real World 30[mm]

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> _exc_jacobian;

        // Experimentation
        double _total_calculation_time = 0.0;
        Eigen::Matrix<double, 6, 1> _pose_error;
        int _test_number = 0;
        ros::Time _calculation_time_start, _calculation_time_end;
        int _calculation_number = 0;
        bool _singular_configuration = false;
        bool _within_angle_limit = true;

    public:
        TreeExCArm();

        void setJoint();

        // Debug
        void print();

        // Subscribe
        void setMotorEnable(std_msgs::Bool motor_enable_);
        void setEmergencyStop(std_msgs::Bool emergency_stop_);
        void setCalculationMode(std_msgs::Int16 calculation_mode_);
        void setSensorAngle(std_msgs::Float32MultiArray sensor_angle_);
        void setTargetAngle(std_msgs::Float32MultiArray target_angle_);
        void setTargetPose(geometry_msgs::Pose target_pose_);
            void setTargetPoseStart();

        // Forward Kinematics
        Eigen::Matrix<double, 6, 1> getPose();

        // Linear Interpolation
        Eigen::Matrix<double, 6, 1> getMidTargetPoseLinearInterpolation();

        // Publish
        bool getMotorEnable();
        bool getEmergencyStop();
        int getCalculationMode();
        std_msgs::Float32MultiArray getMotorAngularVelocityZero();
        std_msgs::Float32MultiArray getMotorAngularVelocity();
            void changeMotorAngularVelocity();
                bool isWithinEmergencyAngleLimit();
                void setMotorAngularVelocityZero();
                void getMotorAngularVelocityByAngle();
                void getMotorAngularVelocityByTimeDiff();
                void getMotorAngularVelocityByExC();

        // TimeDiff: Time Differentiation
        Eigen::Matrix<double, 6, 6> getTimeDiffJacobian();
            Eigen::Matrix<double, 3, 6> getTranslationJacobian();
            Eigen::Matrix<double, 3, 6> getRotationJacobian();
            void replaceVariables();

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobian();
        Eigen::Matrix<double, 6, JOINT_NUMBER> getExCJacobianBody();
            Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 6, 6> adjointInverse(Eigen::Matrix<double, 4, 4> matrix);
            Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> vector);
        Eigen::Matrix<double, 6, 6> getTransformationMatrix();
            Eigen::Matrix<double, 3, 3> getTransformationEuler();
};

TreeExCArm::TreeExCArm()
{
    setJoint();
}

void TreeExCArm::setJoint()
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

// Debug
void TreeExCArm::print()
{
    std::cout

    // << std::endl
    // << "angle"
    // << std::endl
    // << _sensor_angle

    // << std::endl
    // << "pose"
    // << std::endl
    // << getPose()

    << std::endl;
}

// Subscribe
void TreeExCArm::setMotorEnable(std_msgs::Bool motor_enable_)
{
    _motor_enable = motor_enable_.data;
}

void TreeExCArm::setEmergencyStop(std_msgs::Bool emergency_stop_)
{
    _emergency_stop = emergency_stop_.data;
}

void TreeExCArm::setCalculationMode(std_msgs::Int16 calculation_mode_)
{
    _calculation_mode_old = _calculation_mode;
    _calculation_mode = calculation_mode_.data;
}

void TreeExCArm::setSensorAngle(std_msgs::Float32MultiArray sensor_angle_)
{
    sensor_angle_.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _sensor_angle(i,0) = sensor_angle_.data[i];
    }
}

void TreeExCArm::setTargetAngle(std_msgs::Float32MultiArray target_angle_)
{
    target_angle_.data.resize(JOINT_NUMBER);
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _target_angle(i,0) = target_angle_.data[i];
    }
}

void TreeExCArm::setTargetPose(geometry_msgs::Pose target_pose_)
{
    if((target_pose_ != _target_pose_old) || (_calculation_mode != _calculation_mode_old))
    {
        _target_object_pose(0,0) = target_pose_.position.x;
        _target_object_pose(1,0) = target_pose_.position.y;
        _target_object_pose(2,0) = target_pose_.position.z;
        _target_object_pose(3,0) = target_pose_.orientation.z;
        _target_object_pose(4,0) = target_pose_.orientation.y;
        _target_object_pose(5,0) = target_pose_.orientation.x;

        for(int i = 0; i < CHAIN_NUMBER; i++)
        {
            _target_pose.block(6*i, 0, 3, 1) += target_pose_.orientation.w*(tree_base.getRotationMatrixZ(_target_object_pose(3,0))*tree_base.getRotationMatrixY(_target_object_pose(4,0))*tree_base.getRotationMatrixX(_target_object_pose(5,0))*(tree_property.getToolDefaultPose(i)).block(0,0,3,1));
        }

        setTargetPoseStart();
    }

    _target_pose_old = target_pose_;
}

void TreeExCArm::setTargetPoseStart()
{
    _target_pose_start = _pose;
    _time_start_move = ros::Time::now();
    _duration_time = ((_target_pose-_target_pose_start).norm())/_linear_velocity;
}
#endif