#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_Claw
{
public:
    AP_Claw();

    /* Do not allow copies */
    AP_Claw(const AP_Claw &other) = delete;
    AP_Claw &operator=(const AP_Claw &) = delete;

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager &serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    uint8_t data_to_send[50];

    void send_switch(uint8_t is_open);

private:
    AP_HAL::UARTDriver *_port; // UART used to send data to receiver
};