#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL_Boards.h>

class AP_RoboCAN_Driver : public CANSensor
{
public:
    
    AP_RoboCAN_Driver();

    // called from SRV_Channels
    void update(const uint8_t num_poles);

private:

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    // bool send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data = nullptr, const uint8_t data_len = 0);

    void loop();
};

class AP_RoboCAN {
public:
    AP_RoboCAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RoboCAN);

    // static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    static AP_RoboCAN *get_singleton() { return _singleton; }

private:
    static AP_RoboCAN *_singleton;

    AP_Int8 _num_poles;
    AP_RoboCAN_Driver *_driver;
};
namespace AP {
    AP_RoboCAN *robocan();
};

