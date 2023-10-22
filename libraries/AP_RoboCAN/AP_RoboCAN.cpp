#include "AP_RoboCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;


void AP_RoboCAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::RoboCAN) {
            _driver = new AP_RoboCAN_Driver();
            return;
        }
    }
}


AP_RoboCAN_Driver::AP_RoboCAN_Driver() : CANSensor("RoboCAN")
{
    register_driver(AP_CAN::Protocol::RoboCAN);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_RoboCAN_Driver::loop, void), "robocan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}


// parse inbound frames
void AP_RoboCAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }
}


void AP_RoboCAN::update()
{

}

void AP_RoboCAN_Driver::loop()
{     while (true) {   
     hal.scheduler->delay_microseconds(2500); // 400Hz

}

}


namespace AP {
AP_RoboCAN *robocan()
{
    return AP_RoboCAN::get_singleton();
}
};