/**
* @file dynamixel_wrapper_config.h
* @brief
* @author Shunya Hara -> Akiro Harada
* @date 2022.7.29 -> 2022.8.12
* @details
*/
#pragma once
#include <string>
namespace dynamixel_wrapper{
struct dynamixel_wrapper_config_item
{
    dynamixel_wrapper_config_item(){}
    dynamixel_wrapper_config_item(int a, int s):address(a),size(s){}
    int address;
    int size;
};
using item = dynamixel_wrapper_config_item;
struct dynamixel_wrapper_config
{
    item drive_mode;
    item operating_mode;
    item homing_offset;
    item moving_threshold;
    item max_voltage_limit;
    item min_voltage_limit;
    item current_limit;
    item acceleration_limit;
    item velocity_limit;
    item max_position_limit;
    item min_position_limit;
    item torque_enable;
    item led_red;
    item led_green;
    item led_blue;
    item hardware_error_status;
    item velocity_i_gain;
    item velocity_p_gain;
    item position_d_gain;
    item position_i_gain;
    item position_p_gain;
    item goal_current;
    item goal_velocity;
    item profile_acceleration;
    item profile_velocity;
    item goal_position;
    item moving;
    item moving_status;
    item present_current;
    item present_velocity;
    item present_position;
    item present_temperature;
    uint resolution;
    double velocity_scaling_factor;
    double voltage_scaling_factor;
    double current_scaling_factor;
    double accleration_scaling_factor;
    std::string errorCode[8];
};

} // namespace dynamixel_wrapper












