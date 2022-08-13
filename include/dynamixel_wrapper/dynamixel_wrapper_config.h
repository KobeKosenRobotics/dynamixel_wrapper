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
dynamixel_wrapper_config PH42_020_S300_R
{
    /* Drive Mode */
    item(10, 1),
    /* Operating Mode */
    item(11, 1),
    /* Homing Offset */
    item(20, 4),
    /* Moving Threshold */
    item(24, 4),
    /* Max Voltage Limit */
    item(32, 2),
    /* Min Voltage Limit */
    item(34, 2),
    /* Current Limit */
    item(38, 2),
    /* Acceleration Limit */
    item(40, 4),
    /* Velocity Limit */
    item(44, 4),
    /* Max Position Limit */
    item(48, 4),
    /* Min Position Limit */
    item(52, 4),
    /* Torque Enable */
    item(512, 1),
    /* LED Red */
    item(513, 1),
    /* LED Green */
    item(514, 1),
    /* LED Blue */
    item(515, 1),
    /* Hardware Error Status */
    item(518, 1),
    /* Velocity I Gain */
    item(524, 2),
    /* Velocity P Gain */
    item(526, 2),
    /* Position D Gain */
    item(528, 2),
    /* Position I Gain */
    item(530, 2),
    /* Position P Gain */
    item(532, 2),
    /* Goal Current */
    item(550, 2),
    /* Goal Velocity */
    item(552, 4),
    /* Profile Acceleration */
    item(556, 4),
    /* Profile Velocity */
    item(560, 4),
    /* Goal Position */
    item(564, 4),
    /* Moving */
    item(570, 1),
    /* Moving Status */
    item(571, 1),
    /* Present Current */
    item(574, 2),
    /* Present Velocity */
    item(576, 4),
    /* Present Position */
    item(580, 4),
    /* Present Temperature */
    item(594, 1),
    /* Resolution */
    607500,
    /* Velocity Scaling Factor */
    0.01,
    /* Voltage Scaling Factor */
    0.1,
    /* Current Scaling Factor */
    1,
    /* Accerleration Scaling Factor */
    58000,
    /* Hardware Error Status to String*/
    {"Input voltage error", "Motor hall sensor error", "Overheating error", "Motor encorder error", "Electrical shock error", "Overload error", "", ""}
};
dynamixel_wrapper_config PH54_100_S500_R
{
    /* Drive Mode */
    item(10, 1),
    /* Operating Mode */
    item(11, 1),
    /* Homing Offset */
    item(20, 4),
    /* Moving Threshold */
    item(24, 4),
    /* Max Voltage Limit */
    item(32, 2),
    /* Min Voltage Limit */
    item(34, 2),
    /* Current Limit */
    item(38, 2),
    /* Acceleration Limit */
    item(40, 4),
    /* Velocity Limit */
    item(44, 4),
    /* Max Position Limit */
    item(48, 4),
    /* Min Position Limit */
    item(52, 4),
    /* Torque Enable */
    item(512, 1),
    /* LED Red */
    item(513, 1),
    /* LED Green */
    item(514, 1),
    /* LED Blue */
    item(515, 1),
    /* Hardware Error Status */
    item(518, 1),
    /* Velocity I Gain */
    item(524, 2),
    /* Velocity P Gain */
    item(526, 2),
    /* Position D Gain */
    item(528, 2),
    /* Position I Gain */
    item(530, 2),
    /* Position P Gain */
    item(532, 2),
    /* Goal Current */
    item(550, 2),
    /* Goal Velocity */
    item(552, 4),
    /* Profile Acceleration */
    item(556, 4),
    /* Profile Velocity */
    item(560, 4),
    /* Goal Position */
    item(564, 4),
    /* Moving */
    item(570, 1),
    /* Moving Status */
    item(571, 1),
    /* Present Current */
    item(574, 2),
    /* Present Velocity */
    item(576, 4),
    /* Present Position */
    item(580, 4),
    /* Present Temperature */
    item(594, 1),
    /* Resolution */
    1003846,
    /* Velocity Scaling Factor */
    0.01,
    /* Voltage Scaling Factor */
    0.1,
    /* Current Scaling Factor */
    1,
    /* Accerleration Scaling Factor */
    58000,
    /* Hardware Error Status to String*/
    {"Input voltage error", "Motor hall sensor error", "Overheating error", "Motor encorder error", "Electrical shock error", "Overload error", "", ""}
};
dynamixel_wrapper_config PH54_200_S500_R
{
    /* Drive Mode */
    item(10, 1),
    /* Operating Mode */
    item(11, 1),
    /* Homing Offset */
    item(20, 4),
    /* Moving Threshold */
    item(24, 4),
    /* Max Voltage Limit */
    item(32, 2),
    /* Min Voltage Limit */
    item(34, 2),
    /* Current Limit */
    item(38, 2),
    /* Acceleration Limit */
    item(40, 4),
    /* Velocity Limit */
    item(44, 4),
    /* Max Position Limit */
    item(48, 4),
    /* Min Position Limit */
    item(52, 4),
    /* Torque Enable */
    item(512, 1),
    /* LED Red */
    item(513, 1),
    /* LED Green */
    item(514, 1),
    /* LED Blue */
    item(515, 1),
    /* Hardware Error Status */
    item(518, 1),
    /* Velocity I Gain */
    item(524, 2),
    /* Velocity P Gain */
    item(526, 2),
    /* Position D Gain */
    item(528, 2),
    /* Position I Gain */
    item(530, 2),
    /* Position P Gain */
    item(532, 2),
    /* Goal Current */
    item(550, 2),
    /* Goal Velocity */
    item(552, 4),
    /* Profile Acceleration */
    item(556, 4),
    /* Profile Velocity */
    item(560, 4),
    /* Goal Position */
    item(564, 4),
    /* Moving */
    item(570, 1),
    /* Moving Status */
    item(571, 1),
    /* Present Current */
    item(574, 2),
    /* Present Velocity */
    item(576, 4),
    /* Present Position */
    item(580, 4),
    /* Present Temperature */
    item(594, 1),
    /* Resolution */
    1003846,
    /* Velocity Scaling Factor */
    0.01,
    /* Voltage Scaling Factor */
    0.1,
    /* Current Scaling Factor */
    1,
    /* Accerleration Scaling Factor */
    58000,
    /* Hardware Error Status to String*/
    {"Input voltage error", "Motor hall sensor error", "Overheating error", "Motor encorder error", "Electrical shock error", "Overload error", "", ""}
};
} // namespace dynamixel_wrapper












