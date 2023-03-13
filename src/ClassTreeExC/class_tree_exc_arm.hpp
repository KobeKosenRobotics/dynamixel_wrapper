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
        Eigen::Matrix<double, 6, 1> _target_object_pose;
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> _target_pose;

        // Linear Interpolation
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> _target_pose_start;
        geometry_msgs::Pose _target_pose_old;
        ros::Time _time_start_move;
        double _midpoint, _duration_time, _linear_velocity = 300;    // _liner_velocity[mm/s]    // Real World 30[mm]

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> _exc_jacobian, _exc_jacobian_body;
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 6*CHAIN_NUMBER> _transformation_matrix;

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
            Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> getMidTargetPoseLinearInterpolation();

        // Forward Kinematics
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> getPose();

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

        // ExC: Exponential Coordinates
        Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> getExCJacobian();
        Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> getExCJacobianBody();
        Eigen::Matrix<double, 6, 1> getDagger(int chain_, int joint_);
        Eigen::Matrix<double, 6*CHAIN_NUMBER, 6*CHAIN_NUMBER> getTransformationMatrix();
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

    // TODO: include setJoint()
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

    for(int i = 0; i < JOINT_NUMBER+CHAIN_NUMBER; i++)
    {
        _joint[i].setJoint(i);
    }
}

// Debug
void TreeExCArm::print()
{
    std::cout

    << std::endl
    << "angle"
    << std::endl
    << _sensor_angle

    << std::endl
    << "pose"
    << std::endl
    << getPose()

    << std::endl
    << "target pose"
    << std::endl
    << getExCJacobianBody()

    // << std::endl
    // << "dagger"
    // << std::endl
    // << getExCJacobian()

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
        _joint[i].updateTheta(_sensor_angle(i,0));
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
            _target_pose.block(6*i, 0, 3, 1) = _target_object_pose.block(0,0,3,1) + target_pose_.orientation.w*(tree_base.getRotationMatrixZ(_target_object_pose(3,0))*tree_base.getRotationMatrixY(_target_object_pose(4,0))*tree_base.getRotationMatrixX(_target_object_pose(5,0))*(tree_property.getToolDefaultPose(i)).block(0,0,3,1));
            _target_pose.block(6*i+3, 0, 3, 1) = _target_object_pose.block(3,0,3,1);
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

Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> TreeExCArm::getMidTargetPoseLinearInterpolation()
{
    _midpoint = std::min(std::max((ros::Time::now()-_time_start_move).toSec()/_duration_time, 0.0), 1.0);
    return _midpoint*_target_pose +(1-_midpoint)*_target_pose_start;
}

// Forward Kinematics
Eigen::Matrix<double, 6*CHAIN_NUMBER, 1> TreeExCArm::getPose()
{
    for(int i = 0; i < CHAIN_NUMBER; i++)
    {
        _pose.block(6*i,0,6,1) = tree_base.getPose(_joint[JOINT_NUMBER+i].getGsjTheta());
    }

    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    static tf2::Quaternion q;

    // Pose
    for(int i = 0; i < CHAIN_NUMBER; i ++)
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "arm_base_link";
        std::stringstream ss;
        ss << "pose" << i;
        transformStamped.child_frame_id = ss.str();
        transformStamped.transform.translation.x = _pose(6*i+0,0)/1000.0;
        transformStamped.transform.translation.y = _pose(6*i+1,0)/1000.0;
        transformStamped.transform.translation.z = _pose(6*i+2,0)/1000.0;

        q.setRPY(_pose(6*i+5,0), _pose(6*i+4,0), _pose(6*i+3,0));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        // Target Pose
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "arm_base_link";
        std::stringstream tt;
        tt << "target_pose" << i;
        transformStamped.child_frame_id = tt.str();
        transformStamped.transform.translation.x = _target_pose(6*i+0,0)/1000.0;
        transformStamped.transform.translation.y = _target_pose(6*i+1,0)/1000.0;
        transformStamped.transform.translation.z = _target_pose(6*i+2,0)/1000.0;

        q.setRPY(_target_pose(6*i+5,0), _target_pose(6*i+4,0), _target_pose(6*i+3,0));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }

    return _pose;
}

// Publish
bool TreeExCArm::getMotorEnable()
{
    return _motor_enable;
}

bool TreeExCArm::getEmergencyStop()
{
    return _emergency_stop;
}

int TreeExCArm::getCalculationMode()
{
    return _calculation_mode;
}

std_msgs::Float32MultiArray TreeExCArm::getMotorAngularVelocityZero()
{
    std_msgs::Float32MultiArray motor_angular_velocity_zero_;
    motor_angular_velocity_zero_.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        motor_angular_velocity_zero_.data[i] = 0.0;
    }

    return motor_angular_velocity_zero_;
}

std_msgs::Float32MultiArray TreeExCArm::getMotorAngularVelocity()
{
    changeMotorAngularVelocity();

    std_msgs::Float32MultiArray motor_angular_velocity_;
    motor_angular_velocity_.data.resize(JOINT_NUMBER);

    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        motor_angular_velocity_.data[i] = _motor_angular_velocity(i,0);
    }

    return motor_angular_velocity_;
}

void TreeExCArm::changeMotorAngularVelocity()
{
    if(_emergency_stop)
    {
        setMotorAngularVelocityZero();
        return;
    }
    if(isWithinEmergencyAngleLimit())
    {
        std::cout << "ERROR: emergency angle limit" << std::endl;
        setMotorAngularVelocityZero();
        return;
    }

    if(_calculation_mode == 1)
    {
        // _calculation_number++;
        // _calculation_time_start = ros::Time::now();
        getMotorAngularVelocityByExC();
        // _calculation_time_end = ros::Time::now();
        // _total_calculation_time += (_calculation_time_end - _calculation_time_start).toSec();
        return;
    }

    else
    {
        getMotorAngularVelocityByAngle();
        return;
    }
}

bool TreeExCArm::isWithinEmergencyAngleLimit()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        if((_sensor_angle(i,0) < tree_property.getLowerAngleLimit(i))-0.5 || (tree_property.getUpperAngleLimit(i)+0.5 < _sensor_angle(i,0)))
        {
            return false;
        }
    }

    return true;
}

void TreeExCArm::setMotorAngularVelocityZero()
{
    for(int i = 0; i < JOINT_NUMBER; i++)
    {
        _motor_angular_velocity(i,0) = 0.0;
    }
}

void TreeExCArm::getMotorAngularVelocityByAngle()
{
    getPose();
    _motor_angular_velocity = tree_property.getProportionalGainAngleOperating()*(_target_angle - _sensor_angle);
}

void TreeExCArm::getMotorAngularVelocityByExC()
{
    #ifdef DOF6
    _motor_angular_velocity = _proportional_gain*(getExCJacobian().inverse())*(getMidTargetPoseLinearInterpolation()-getPose());
    #endif

    #ifndef DOF6
    _motor_angular_velocity = tree_property.getProportionalGainExC()*(tree_base.getPseudoInverseMatrix(getExCJacobian()))*(getMidTargetPoseLinearInterpolation()-getPose());
    #endif
}

// ExC: Exponential Coordinates
Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> TreeExCArm::getExCJacobian()
{
    _exc_jacobian = (getTransformationMatrix().inverse())*getExCJacobianBody();

    // if(tree_base.getRank(_exc_jacobian) < JOINT_NUMBER)
    // {
    //     _singular_configuration = true;
    // }

    return _exc_jacobian;
}

Eigen::Matrix<double, 6*CHAIN_NUMBER, JOINT_NUMBER> TreeExCArm::getExCJacobianBody()
{
    for(int i = 0; i < CHAIN_NUMBER; i++)
    {
        for(int j = 0; j < JOINT_NUMBER; j++)
        {
            // _exc_jacobian_body.block(6*i,j,6,1) << _joint[JOINT_NUMBER+i].getXiDagger(i,j);
            // _exc_jacobian_body.block(6*i,j,6,1) = getDagger(i,j);
            if(tree_property.getChainMatrix(i,j)) _exc_jacobian_body.block(6*i,j,6,1) = (tree_base.adjointInverse(_joint[JOINT_NUMBER+i].getChildrenExpXiHatTheta(j)))*(_joint[j].getXi());
        }
    }

    return _exc_jacobian_body;
}

// Eigen::Matrix<double, 6, 1> TreeExCArm::getDagger(int chain_, int joint_)
// {
//     if(!tree_property.getChainMatrix(chain_, joint_)) return Eigen::Matrix<double, 6, 1>::Zero();
//     Eigen::Matrix<double, 4, 4> matrix_;
//     matrix_ = _joint[JOINT_NUMBER+chain_].getGsjZero();
//     // std::cout << matrix_ << std::endl << std::endl;
//     for(int i = JOINT_NUMBER-1; i >= joint_; i--)
//     {
//         // std::cout << _joint[i].getExpXiHatTheta(chain_) << std::endl << std::endl;
//         matrix_ = _joint[i].getExpXiHatTheta(chain_, i)*matrix_;
//     }

//     return tree_base.adjointInverse(matrix_)*_joint[joint_].getXi();
// }

Eigen::Matrix<double, 6*CHAIN_NUMBER, 6*CHAIN_NUMBER> TreeExCArm::getTransformationMatrix()
{
    for(int i = 0; i < CHAIN_NUMBER; i++)
    {
        _transformation_matrix.block(6*i,6*i,3,3) = ((_joint[JOINT_NUMBER+i].getGsjTheta()).block(0,0,3,3)).transpose();
        _transformation_matrix.block(6*i+3,6*i+3,3,3) = tree_base.getTransformationEuler(_pose.block(6*i+3,0,3,1));
    }

    return _transformation_matrix;
}

#endif