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

#include <dynamixel_wrapper/dynamixel_wrapper_base.h>
#include <dynamixel_wrapper/dynamixel_wrapper_configs.h>

namespace dynamixel_wrapper{
enum class Mode:int{
    Current,
    Velocity,
    Position,
    ExtendedPosition,
    CurrentBasePosition,
    PWM
};
class dynamixel_wrapper{
    public:
    dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config, Mode operatingMode);
    dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config, int operatingMode);
    ~dynamixel_wrapper();
    void write(dynamixel_wrapper_config_item item, int value);
    uint32_t read(dynamixel_wrapper_config_item item);
    int32_t read_signed(dynamixel_wrapper_config_item item);
    void setDriveMode(int driveMode) { write(motor_config_.drive_mode, driveMode); }
    void setOperatingMode(Mode operatingMode) { write(motor_config_.operating_mode, static_cast<int>(operatingMode)); }
    void setOperatingMode(int operatingMode) { write(motor_config_.operating_mode, operatingMode); }
    void setHomingOffset(double degree) { write(motor_config_.homing_offset, (int)(degree/360.0f*motor_config_.resolution)); }
    void setMovingThreshold(float rpm){ write(motor_config_.moving_threshold, (int)(rpm/motor_config_.velocity_scaling_factor)); }
    void setMaxVoltageLimit(float voltage) { write(motor_config_.max_voltage_limit, (int)(voltage/motor_config_.voltage_scaling_factor)); }
    void setminVoltageLimit(float voltage) { write(motor_config_.min_position_limit, (int)(voltage/motor_config_.voltage_scaling_factor)); }
    void setCurrentLimit(double mA) { write(motor_config_.current_limit, (int)(mA/motor_config_.current_scaling_factor)); }
    void setAccelerationLimit(float rpm2){ write(motor_config_.acceleration_limit, (int)(rpm2/motor_config_.accleration_scaling_factor)); }
    void setVelocityLimit(double rpm){ write(motor_config_.velocity_limit, (int)(rpm/motor_config_.velocity_scaling_factor)); }
    void setMaxPositionLimit(double degree){ write(motor_config_.max_position_limit, (int)(degree/360.0f*motor_config_.resolution)); }
    void setMinPositionLimit(double degree){ write(motor_config_.min_position_limit, (int)(degree/360.0f*motor_config_.resolution)); }
    void setTorqueEnable(bool state){ write(motor_config_.torque_enable, (int)state); }
    void setLED(int r, int g, int b)
    {
        write(motor_config_.led_red, r);
        write(motor_config_.led_green, g);
        write(motor_config_.led_blue, b);
    }
    std::string getHardwareErrorStatus()
    {
        std::string errorCode = "";
        uint32_t errorNum = read(motor_config_.hardware_error_status);
        for(int i=0;i<8;i++)
        {
            if((errorNum >> i)&0x01)errorCode += (motor_config_.errorCode[i]+" ");
        }
        if(errorCode=="")
            errorCode+="No Error";
        return errorCode;
    }
    void setVelocityGain(int Kp, int Ki)
    {
        write(motor_config_.velocity_p_gain, Kp);
        write(motor_config_.velocity_i_gain, Ki);
    }
    void setPositionGain(int Kp, int Ki, int Kd)
    {
        write(motor_config_.position_p_gain, Kp);
        write(motor_config_.position_i_gain, Ki);
        write(motor_config_.position_d_gain, Kd);
    }
    void setGoalCurrent(double mA){ write(motor_config_.goal_current, (int)(mA/motor_config_.current_scaling_factor)); }
    void setGoalVelocity(double rpm) { write(motor_config_.goal_velocity, (int)(rpm/motor_config_.velocity_scaling_factor)); }
    void setProfileAcceleration(double rpm2){ write(motor_config_.profile_acceleration, (int)(rpm2/motor_config_.accleration_scaling_factor)); }
    void setProfileVelocity(double rpm){ write(motor_config_.profile_velocity, (int)(rpm/motor_config_.velocity_scaling_factor)); }
    void setGoalPosition(double degree){ write(motor_config_.goal_position, (int)(degree/360.0f*motor_config_.resolution)); }
    bool getMoving(){ return read(motor_config_.moving); }
    bool getIsInPosition(){return (read(motor_config_.moving_status)&0x01); }
    double getPresentCurrent(){ return read_signed(motor_config_.present_current)*motor_config_.current_scaling_factor; }
    double getPresentVelocity(){ return read_signed(motor_config_.present_velocity)*motor_config_.velocity_scaling_factor; }
    double getPresentPosition(){ return read_signed(motor_config_.present_position)*360.0f/motor_config_.resolution; }
    double getGoalCurrent(){ return read_signed(motor_config_.goal_current)*motor_config_.current_scaling_factor; }
    double getGoalVelocity(){ return read_signed(motor_config_.goal_velocity)*motor_config_.velocity_scaling_factor; }
    double getGoalPosition(){ return read_signed(motor_config_.goal_position)*360.0f/motor_config_.resolution; }
    double getPresentTemperature(){ return read(motor_config_.present_temperature); }
    private:
    int id_;
    dynamixel_wrapper_base* dxl_base_;
    dynamixel_wrapper_config motor_config_;
};
dynamixel_wrapper::dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config, Mode operatingMode):
dynamixel_wrapper(id,dxl_base,motor_config,static_cast<int>(operatingMode))
{
}
dynamixel_wrapper::dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config, int operatingMode){
    id_=id;
    dxl_base_= &dxl_base;
    motor_config_=motor_config;
    setTorqueEnable(false);
    setOperatingMode(operatingMode);
    setTorqueEnable(true);
}
dynamixel_wrapper::~dynamixel_wrapper(){
    setTorqueEnable(false);
}
void dynamixel_wrapper::write(dynamixel_wrapper_config_item item, int value){
    int address = item.address;
    int byte_size = item.size;
    //ROS_INFO("%d, %d, %d", address, byte_size, value);
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

uint32_t dynamixel_wrapper::read(dynamixel_wrapper_config_item item){
    int address = item.address;
    int byte_size = item.size;
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

int32_t dynamixel_wrapper::read_signed(dynamixel_wrapper_config_item item){
    int address = item.address;
    int byte_size = item.size;
    uint8_t value8;
    uint16_t value16;
    uint32_t value32;
    int32_t result=0;
    if(byte_size==1){
        dxl_base_->packetHandler->read1ByteTxRx(dxl_base_->portHandler, id_, address, &value8);
        result=value8;
        result=result>INT8_MAX?(result-UINT8_MAX):result;
    }
    else if(byte_size==2){
        dxl_base_->packetHandler->read2ByteTxRx(dxl_base_->portHandler, id_, address, &value16);
        result=value16;
        result=result>INT16_MAX?(result-UINT16_MAX):result;
    }
    else if(byte_size==4){
        dxl_base_->packetHandler->read4ByteTxRx(dxl_base_->portHandler, id_, address, &value32);
        result=value32;
        result=result>INT32_MAX?(result-UINT32_MAX):result;
    }
    else{
        ROS_ERROR("Byte size is undefined");
        return 0;
    }
    return result;
}
}//namespace dynamixel_wrapper