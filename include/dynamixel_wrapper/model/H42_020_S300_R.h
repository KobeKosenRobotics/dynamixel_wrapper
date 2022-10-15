#pragma once

#include <dynamixel_wrapper/dynamixel_wrapper_config.h>

namespace dynamixel_wrapper{
dynamixel_wrapper_config H42_020_S300_R
{
    /* Drive Mode */
    item(0, 0),    // Not Exist
    /* Operating Mode */
    item(11, 1),
    /* Homing Offset */
    item(13, 4),
    /* Moving Threshold */
    item(17, 4),
    /* Max Voltage Limit */
    item(22, 2),
    /* Min Voltage Limit */
    item(24, 2),
    /* Current Limit */
    item(0, 0),    // Not Exist
    /* Acceleration Limit */
    item(26, 4),
    /* Velocity Limit */
    item(32, 4),
    /* Max Position Limit */
    item(36, 4),
    /* Min Position Limit */
    item(40, 4),
    /* Torque Enable */
    item(562, 1),
    /* LED Red */
    item(563, 1),
    /* LED Green */
    item(564, 1),
    /* LED Blue */
    item(565, 1),
    /* Hardware Error Status */
    item(892, 1),
    /* Velocity I Gain */
    item(586, 2),
    /* Velocity P Gain */
    item(588, 2),
    /* Position D Gain */
    item(0, 0),    // Not Exist
    /* Position I Gain */
    item(0, 0),    // Not Exist
    /* Position P Gain */
    item(594, 2),
    /* Goal Current */
    item(0, 0),    // Not Exist
    /* Goal Velocity */
    item(600, 4),
    /* Profile Acceleration */
    item(0, 0),
    /* Profile Velocity */
    item(0, 0),
    /* Goal Position */
    item(596, 4),
    /* Moving */
    item(610, 1),
    /* Moving Status */
    item(0, 0),    // Not Exist
    /* Present Current */
    item(621, 2),
    /* Present Velocity */
    item(615, 4),
    /* Present Position */
    item(611, 4),
    /* Present Temperature */
    item(625, 1),
    /* Resolution */
    303750,
    /* Velocity Scaling Factor */
    1/303.750,
    /* Voltage Scaling Factor */
    0.0,    // Not Exist
    /* Current Scaling Factor */
    8250/2048,
    /* Accerleration Scaling Factor */
    0.0,
    /* Hardware Error Status to String*/
    {"Input voltage error", "Motor hall sensor error", "Overheating error", "Motor encorder error", "Electrical shock error", "Overload error", "", ""}
};
} // namespace dynamixel_wrapper