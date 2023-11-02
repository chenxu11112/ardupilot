#pragma once

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_EFI/AP_EFI_Currawong_ECU.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_RoboCAN : public AP_CANDriver {
public:
    AP_RoboCAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RoboCAN);

    static const struct AP_Param::GroupInfo var_info[];

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    int16_t getSpeed(uint8_t id);

    void setCurrent(uint8_t id, int16_t _cuur)
    {
        if (id < 1) {
            return;
        }
        target_current[id - 1] = _cuur;
    }

    static AP_RoboCAN* get_singleton() { return _singleton; }

    bool read_frame(AP_HAL::CANFrame& recv_frame, uint64_t timeout);
    bool write_frame(AP_HAL::CANFrame& out_frame, uint64_t timeout);

    void loop();

private:
    static AP_RoboCAN* _singleton;

    /*CAN发送或是接收的ID*/
    typedef enum {

        CAN_2006Moto_ALL_ID = 0x200,
        CAN_2006Moto1_ID    = 0x201,
        CAN_2006Moto2_ID    = 0x202,
        CAN_2006Moto3_ID    = 0x203,
        CAN_2006Moto4_ID    = 0x204,

    } CAN_Message_ID;

    AP_Int8 _num_poles;

    AP_HAL::CANIface* _can_iface;
    uint8_t           _driver_index;
    bool              _initialized;
    char              _thread_name[16];

    void handle_moto_measure(AP_HAL::CANFrame& frame, uint8_t id);
    bool send_current(const int16_t d1, const int16_t d2, const int16_t d3, const int16_t d4);

    int16_t target_current[4];
    int16_t real_speed[4];
    uint8_t send_current_buffer[8];
};
namespace AP {
AP_RoboCAN* robocan();
};
