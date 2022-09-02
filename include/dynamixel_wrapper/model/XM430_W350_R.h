#pragma once

#include <dynamixel_wrapper/dynamixel_wrapper_config.h>

namespace dynamixel_wrapper{
dynamixel_wrapper_config XM430_W350_R
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
    item(64, 1),
    /* LED Red */
    item(65, 1),
    /* LED Green */
    item(65, 1),
    /* LED Blue */
    item(65, 1),
    /* Hardware Error Status */
    item(70, 1),
    /* Velocity I Gain */
    item(76, 2),
    /* Velocity P Gain */
    item(78, 2),
    /* Position D Gain */
    item(80, 2),
    /* Position I Gain */
    item(82, 2),
    /* Position P Gain */
    item(84, 2),
    /* Goal Current */
    item(102, 2),
    /* Goal Velocity */
    item(104, 4),
    /* Profile Acceleration */
    item(108, 4),
    /* Profile Velocity */
    item(112, 4),
    /* Goal Position */
    item(116, 4),
    /* Moving */
    item(122, 1),
    /* Moving Status */
    item(123, 1),
    /* Present Current */
    item(126, 2),
    /* Present Velocity */
    item(128, 4),
    /* Present Position */
    item(132, 4),
    /* Present Temperature */
    item(146, 1),
    /* Resolution */
    4096,
    /* Velocity Scaling Factor */
    0.229,
    /* Voltage Scaling Factor */
    0.1,
    /* Current Scaling Factor */
    2.69,
    /* Accerleration Scaling Factor */
    214.577,
    /* Hardware Error Status to String*/
    {"Input voltage error", "Motor hall sensor error", "Overheating error", "Motor encorder error", "Electrical shock error", "Overload error", "", ""}
};
} // namespace dynamixel_wrapper