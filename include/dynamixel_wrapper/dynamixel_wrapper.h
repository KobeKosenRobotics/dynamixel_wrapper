/**
* @file dynamixel_wrapper.h
* @brief easy use for dynamixel_sdk
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#pragma once

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <dynamixel_sdk/dynamixel_sdk.h>
//#include <dynamixel_wrapper/dynamixel_wrapper_base.h>
//#include <dynamixel_wrapper/dynamixel_wrapper_config.h>
#include "dynamixel_wrapper_base.h"
#include "dynamixel_wrapper_config.h"

namespace dynamixel_wrapper{

class dynamixel_wrapper{
    public:
    dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config,double current_limit=10000.0);
    void write(int address,int byte_size, int value);
    uint32_t read(int address,int byte_size);
    int32_t read_signed(int address,int byte_size);

    void setGoalPosition(double deg){write(motor_config_.goal_position,motor_config_.goal_position_size,deg*4096.0/360.0);}
    double getGoalPosition(){return int(read(motor_config_.goal_position,motor_config_.goal_position_size))*360/4096.0;}

    void setGoalCurrent(double current/*[mA]*/){write(motor_config_.goal_current,motor_config_.goal_current_size,current/*[mA]*/);}
    double getGoalCurrent(){return int(read(motor_config_.goal_current,motor_config_.goal_current_size));}

    //if toqque ebable, no working
    void setCurrentLimit(double current/*[mA]*/){write(motor_config_.current_limit,motor_config_.current_limit_size,int(current/motor_config_.current_scaling_factor));}
    double getCurrentLimit(){return read(motor_config_.current_limit,motor_config_.current_limit_size)*motor_config_.current_scaling_factor;}

    void setTorqueEnable(bool is_enable){write(motor_config_.torque_enable,motor_config_.torque_enable_size,is_enable);}
    bool getTorqueEnable(){return read(motor_config_.torque_enable,motor_config_.torque_enable_size);}

    void setOperatingMode(int mode){write(motor_config_.operating_mode,motor_config_.operating_mode_size,mode);}
    int getOperatingMode(){return read(motor_config_.operating_mode,motor_config_.operating_mode_size);}

    double getCurrentPosition(){return int(read(motor_config_.current_position,motor_config_.current_position_size))*360/4096.0;}

    double getCurrentCurrent(){return int(read_signed(motor_config_.current_current,motor_config_.current_current_size))*motor_config_.current_scaling_factor;}

    private:
    int id_;
    dynamixel_wrapper_base* dxl_base_;
    dynamixel_wrapper_config motor_config_;
};




dynamixel_wrapper::dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config,double current_limit){
    id_=id;
    dxl_base_= &dxl_base;
    motor_config_=motor_config;

    setTorqueEnable(false);
    // initialize
    setCurrentLimit(current_limit);
    //setOperatingMode(5);

    setTorqueEnable(true);
}

void dynamixel_wrapper::write(int address,int byte_size, int value){
    bool dxl_comm_result;
    if(byte_size==1){
        dxl_comm_result = dxl_base_->packetHandler->write1ByteTxRx(dxl_base_->portHandler, id_, address, value);
        //std::cout<<value<<std::endl;
    }
    else if(byte_size==2){
        dxl_comm_result = dxl_base_->packetHandler->write2ByteTxRx(dxl_base_->portHandler, id_, address, value);
    }
    else if(byte_size==4){
        dxl_comm_result = dxl_base_->packetHandler->write4ByteTxRx(dxl_base_->portHandler, id_, address, value);
    }
    else{
        ROS_ERROR("Byte size is undefined");
    }
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to connect for Dynamixel ID %d", id_);
    }
}

uint32_t dynamixel_wrapper::read(int address,int byte_size){
    uint8_t value8;
    uint16_t value16;
    uint32_t value32;
    if(byte_size==1){
        dxl_base_->packetHandler->read1ByteTxRx(dxl_base_->portHandler, id_, address, &value8);
        value32=value8;
    }
    else if(byte_size==2){
        dxl_base_->packetHandler->read2ByteTxRx(dxl_base_->portHandler, id_, address, &value16);
        value32=value16;
    }
    else if(byte_size==4){
        dxl_base_->packetHandler->read4ByteTxRx(dxl_base_->portHandler, id_, address, &value32);
    }
    else{
        ROS_ERROR("Byte size is undefined");
        return 0;
    }
    
    return value32;
}

int32_t dynamixel_wrapper::read_signed(int address,int byte_size){
    uint8_t value8;
    uint16_t value16;
    uint32_t value32;
    int32_t result=0;
    if(byte_size==1){
        dxl_base_->packetHandler->read1ByteTxRx(dxl_base_->portHandler, id_, address, &value8);
        result=value8;
    }
    else if(byte_size==2){
        dxl_base_->packetHandler->read2ByteTxRx(dxl_base_->portHandler, id_, address, &value16);
        result=value16;
        result=result>INT16_MAX?(result-UINT16_MAX):result;
    }
    else if(byte_size==4){
        dxl_base_->packetHandler->read4ByteTxRx(dxl_base_->portHandler, id_, address, &value32);
        result=value32;
    }
    else{
        ROS_ERROR("Byte size is undefined");
        return 0;
    }
    
    return result;
}

}//namespace dynamixel_wrapper