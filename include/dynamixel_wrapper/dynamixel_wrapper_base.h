/**
* @file dynamixel_wrapper_base.h
* @brief easy use for dynamixel_sdk
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#pragma once

#include <ros/ros.h>
#include <string>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynamixel_wrapper{

class dynamixel_wrapper_base{
    public:
    dynamixel_wrapper_base(const std::string& port_name, const int& baudrate, const float& protcol_version=2.0);
    dynamixel_wrapper_base(){}
    void initialize(const std::string& port_name, const int& baudrate, const float& protcol_version=2.0);

    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;
};

dynamixel_wrapper_base::dynamixel_wrapper_base(const std::string& port_name, const int& baudrate, const float& protcol_version){
    portHandler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protcol_version);
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
    }

    if (!portHandler->setBaudRate(baudrate)) {
        ROS_ERROR("Failed to set the baudrate!");
    }
}

void dynamixel_wrapper_base::initialize(const std::string& port_name, const int& baudrate, const float& protcol_version)
{
    portHandler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protcol_version);
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
    }

    if (!portHandler->setBaudRate(baudrate)) {
        ROS_ERROR("Failed to set the baudrate!");
    }
}

}//namespace dynamixel_wrapper