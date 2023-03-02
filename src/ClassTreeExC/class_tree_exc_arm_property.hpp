#ifndef CLASS_TREE_EXC_ARM_PROPERTY_HPP
#define CLASS_TREE_EXC_ARM_PROPERTY_HPP

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

const int JOINT_NUMBER = 6;
const int CHAIN_NUMBER = 2;

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);

class TreeExCArmProperty
{
    private:
        Eigen::Matrix<double, JOINT_NUMBER+CHAIN_NUMBER, 3> _link;
        Eigen::Matrix<double, 3, JOINT_NUMBER+CHAIN_NUMBER> _joint_position;
        Eigen::Matrix<double, 3, JOINT_NUMBER> _translation_axis, _rotation_axis;
        Eigen::Matrix<std::string, JOINT_NUMBER+CHAIN_NUMBER, 1> _joint_name;
        Eigen::Matrix<double, JOINT_NUMBER, JOINT_NUMBER> _proportional_gain_angle_operating;
        Eigen::Matrix<double, JOINT_NUMBER, 1> _initial_target_angle;
        Eigen::Matrix<double, JOINT_NUMBER, 2> _joint_angle_limit;
        Eigen::Matrix<int, CHAIN_NUMBER, JOINT_NUMBER> _chain_matrix;

    public:
        TreeExCArmProperty();
};
TreeExCArmProperty tree_property;

TreeExCArmProperty::TreeExCArmProperty()
{
}


#endif

// #include <iostream>
// #include <utility>
  
// int count = 0;
// int joint, parent;
// int children[] = {-1, -1, -1, -1, -1, -1, -1, -1};

// // パラメータパックが空になったら終了
// void print()
// {
//   count = 0;
// }

// // ひとつ以上のパラメータを受け取るようにし、
// // 可変引数を先頭とそれ以外に分割する
// template <class Joint, class Parent, class Head, class... Tail>
// void print(Joint&& joint, Parent&& parent, Head&& head, Tail&&... tail)
// {
//   if(count == 0) {joint = head;}
//   else if(count == 1) {parent = head;}
//   else if(head > 0){children[count-2] = head;}
  
  
//   count++;
  
//   // std::cout << head << std::endl;

//   // パラメータパックtailをさらにheadとtailに分割する
//   print(std::forward<Tail>(tail)...);
// }

// int main()
// {
//   print(3, 2, 5, 6, 7);
  
//   std::cout << "joint: " << joint << std::endl;
//   std::cout << "parent: " << parent << std::endl;
//   for(int i = 0; i < 6; i++)
//   {
//     if(i == 0) {std::cout << "children: ";}
//     if(children[i] >= 0) {std::cout << children[i] << " ";}
//   }
//   std::cout << std::endl;
// }